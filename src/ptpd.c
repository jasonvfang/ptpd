/*-
 * Copyright (c) 2012-2013 Wojciech Owczarek,
 * Copyright (c) 2011-2012 George V. Neville-Neil,
 *                         Steven Kreuzer,
 *                         Martin Burnicki,
 *                         Jan Breuer,
 *                         Gael Mace,
 *                         Alexandre Van Kempen,
 *                         Inaqui Delgado,
 *                         Rick Ratzel,
 *                         National Instruments.
 * Copyright (c) 2009-2010 George V. Neville-Neil,
 *                         Steven Kreuzer,
 *                         Martin Burnicki,
 *                         Jan Breuer,
 *                         Gael Mace,
 *                         Alexandre Van Kempen
 *
 * Copyright (c) 2005-2008 Kendall Correll, Aidan Williams
 *
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file   ptpd.c
 * @date   Wed Jun 23 10:13:38 2010
 *
 * @brief  The main() function for the PTP daemon
 *
 * This file contains very little code, as should be obvious,
 * and only serves to tie together the rest of the daemon.
 * All of the default options are set here, but command line
 * arguments and configuration file is processed in the
 * ptpdStartup() routine called
 * below.
 */

#include "ptpd.h"

RunTimeOpts rtOpts;			/* statically allocated run-time
					 * configuration data */

 /*
  * Global variable with the main PTP port. This is used to show the current state in DBG()/message()
  * without having to pass the pointer everytime.
  *
  * if ptpd is extended to handle multiple ports (eg, to instantiate a Boundary Clock),
  * then DBG()/message() needs a per-port pointer argument
  */
 PtpClock *G_ptpClock = NULL;
                     
#ifdef APTP

PTPSharedInternalData *gAptpShm = NULL;
int gShmId = 0;
int gMsgFifoFd = -1;

PTPSharedInternalData* PTPClockShmGet(void)
{
	PTPSharedInternalData* shmem = NULL;
	int shmid=shmget( APTP_SHM_ID, 0, 0);
	if(shmid != -1)
		shmem = shmat( shmid,( const void* )0,0 );
	return shmem;
}


PTPSharedInternalData* PTPClockShmInit(void)
{
    PTPSharedInternalData* shmem = NULL;
    int shmid=shmget( APTP_SHM_ID, sizeof(PTPSharedInternalData)+128, 0666 | IPC_CREAT | IPC_EXCL );

    if (shmid != -1)
    {
        shmem = shmat( shmid, ( const void* )0,0 );
        if(shmem)
        {
            memset(shmem, 0, sizeof(PTPSharedInternalData));
            shmem->size = sizeof(PTPSharedInternalData);
        }
    }
    else
    {
        if(EEXIST == errno || EINVAL == errno)
        {
            shmid=shmget( APTP_SHM_ID, 0, 0);
            if(shmid != -1)
                shmem = shmat( shmid,( const void* )0,0 );
        }
    }
    
    gShmId = shmid;

    if (shmem == NULL)
    {
        DBG( "%s,%d: fail of get share memory\n", __func__, __LINE__);
        return NULL;
    }

    return shmem;
}

int init_fifo_msg_comm(void)
{
    int res = 0;

    DBG("Aptpd Create the fifo pipe %s.\n", APTPD_MSG_FIFO_NAME);

    unlink(APTPD_MSG_FIFO_NAME);
    
    res = mkfifo(APTPD_MSG_FIFO_NAME, O_CREAT |0666);
    if(res != 0)
    {
        DBG("Could not create fifo %s\n", APTPD_MSG_FIFO_NAME);
        return -1;
    }

    return res;
}


int ptpd_msg_handle_routine(MessageData *inMsgData)
{
    int ret = 0;
    
    if (inMsgData == NULL)
        return -1;

    switch (inMsgData->opCode)
    {
        case AddressApiAdd:
        {
            DBG(USER_DESCRIPTION" recvd peer add msg, peer addr=[%s]\n", inMsgData->ipAddress);

            G_ptpClock->rtOpts->ipMode = IPMODE_UNICAST;/* switch to unicast mode */

            G_ptpClock->rtOpts->unicastDestinationsSet = TRUE;

            memset(G_ptpClock->rtOpts->unicastDestinations, 0, MAXHOSTNAMELEN * UNICAST_MAX_DESTINATIONS);
            strncpy(&G_ptpClock->rtOpts->unicastDestinations[0], inMsgData->ipAddress, MAXHOSTNAMELEN);
        }
        break;

        case AddressApiDelete:
        {
            DBG(USER_DESCRIPTION" recvd peer delete msg, peer addr=[%s]\n", inMsgData->ipAddress);
            G_ptpClock->rtOpts->unicastDestinationsSet = FALSE;
            memset(G_ptpClock->rtOpts->unicastDestinations, 0, MAXHOSTNAMELEN * UNICAST_MAX_DESTINATIONS);            
        }
        break;

        default:
            break;
    }


    return ret;
}


void *ptpd_msg_ipc_thread(void *arg)
{
    int max_fd = 0, KeepLoop = 1;
    fd_set rfds;
    MessageData msgBuff;
    
    gMsgFifoFd = open(APTPD_MSG_FIFO_NAME, O_RDWR);
    DBG(USER_DESCRIPTION" open fifo msg fd [%d]\n", gMsgFifoFd);

    if (gMsgFifoFd <= 0)
    {
        DBG(USER_DESCRIPTION" open fifo msg fd err:%d, %s\n", errno, strerror(errno));
        return (void*)0;
    }
    
    while (KeepLoop)
    {
        FD_ZERO(&rfds);
        FD_SET(gMsgFifoFd, &rfds);

        max_fd = gMsgFifoFd;
        
        int retval = select(max_fd + 1, &rfds, NULL, NULL, NULL);

        if (retval > 0)
        {
            if (FD_ISSET(gMsgFifoFd, &rfds))
            {
                memset(&msgBuff, 0, sizeof(msgBuff));
                read(gMsgFifoFd, &msgBuff, sizeof(MessageData));

                ptpd_msg_handle_routine(&msgBuff);
            }
        }
    }

    close(gMsgFifoFd);

    return (void *)0;
}

int ptpd_msg_ipc_thread_init(void)
{
    pthread_t thread_t;

    if (0 != pthread_create(&thread_t, NULL, ptpd_msg_ipc_thread, NULL))
    {
        DBG("create msg thread fifo failed, errno:%d,%s!\n", errno, strerror(errno));
        return errno;
    }
    
    return 0;
}
#endif


Boolean startupInProgress;

TimingDomain timingDomain;

int
main(int argc, char **argv)
{
	PtpClock *ptpClock;
	Integer16 ret;
	TimingService *ts;

	startupInProgress = TRUE;

	memset(&timingDomain, 0, sizeof(timingDomain));
	timingDomainSetup(&timingDomain);

	timingDomain.electionLeft = 10;

#ifdef APTP
      if (init_fifo_msg_comm() != 0)
      {
          ERROR(USER_DESCRIPTION" init FIFO IPC failed!!\n");
          return -1;
      }

      gAptpShm = PTPClockShmInit();
      if (gAptpShm == NULL)
      {
          ERROR(USER_DESCRIPTION" init Shm IPC failed!!\n");
          return -1;
      }

      DBG("Shm init ok, gAptpShm=[0x%x]\n", gAptpShm);

      memset(&gAptpShm->internalData, 0, sizeof(gAptpShm->internalData));
      gAptpShm->clockIdentity = 0;
      gAptpShm->aptpd_launched_flag = APTPD_START_IN_PROGRESS;

      if (0 != ptpd_msg_ipc_thread_init())
      {
          ERROR(USER_DESCRIPTION" msg IPC thread init failed!!\n");          
          return -1;
      }
#endif

	/* Initialize run time options with command line arguments */
	if (!(ptpClock = ptpdStartup(argc, argv, &ret, &rtOpts))) {
		if (ret != 0 && !rtOpts.checkConfigOnly)
			ERROR(USER_DESCRIPTION" startup failed\n");
		return ret;
	}

	timingDomain.electionDelay = rtOpts.electionDelay;

	/* configure PTP TimeService */
	timingDomain.services[0] = &ptpClock->timingService;
	ts = timingDomain.services[0];
	strncpy(ts->id, "PTP0", TIMINGSERVICE_MAX_DESC);
	ts->dataSet.priority1 = rtOpts.preferNTP;
	ts->dataSet.type = TIMINGSERVICE_PTP;
	ts->config = &rtOpts;
	ts->controller = ptpClock;
	ts->timeout = rtOpts.idleTimeout;
	ts->updateInterval = 1;
	ts->holdTime = rtOpts.ntpOptions.failoverTimeout;
	timingDomain.serviceCount = 1;

	if (rtOpts.ntpOptions.enableEngine) {
		ntpSetup(&rtOpts, ptpClock);
	} else {
	    timingDomain.serviceCount = 1;
	    timingDomain.services[1] = NULL;
	}

	timingDomain.init(&timingDomain);
	timingDomain.updateInterval = 1;

	startupInProgress = FALSE;
    
#ifdef APTP
      gAptpShm->aptpd_launched_flag = APTPD_STARTED_OK;
#endif

	/* global variable for message(), please see comment on top of this file */
	G_ptpClock = ptpClock;

	/* do the protocol engine */
	protocol(&rtOpts, ptpClock);
	/* forever loop.. */

	/* this also calls ptpd shutdown */
	timingDomain.shutdown(&timingDomain);

	NOTIFY("Self shutdown\n");

	return 1;
}
