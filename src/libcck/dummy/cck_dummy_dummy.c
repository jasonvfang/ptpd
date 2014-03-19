/*-
 * libCCK - Clock Construction Kit
 *
 * Copyright (c) 2014 Wojciech Owczarek,
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
 * @file   cck_dummy_dummy.c
 *
 * @brief  libCCK dummy component dummy implementation - full implementatione example
 *
 */

#include "cck_dummy_dummy.h"

#define CCK_THIS_TYPE CCK_DUMMY_DUMMY

/* interface (public) method definitions */
static int     cckDummyInit (CckDummy* self, const CckDummyConfig* config);
static int     cckDummyShutdown (void* component);
static CckBool cckDummyDoSomething (CckDummy* self, int param1);
static CckBool cckDummyDoSomethingWithCallback (CckDummy* self, int param1);

/* private method definitions (if any) */
static int     private1(CckDummy* dummy, int* param1);

/* implementations follow */

void
cckDummySetup_dummy(CckDummy* self)
{
	if(self->dummyType == CCK_THIS_TYPE) {
            self->dummyCallback = NULL;
	    self->init = cckDummyInit;
	    self->shutdown = cckDummyShutdown;
	    self->doSomething = cckDummyDoSomething;
	    self->doSomethingWithCallback = cckDummyDoSomethingWithCallback;
	    self->header.shutdown = cckDummyShutdown;
	} else {
	    CCK_WARNING("setup() called for incorrect component implementation: %02x, expected %02x\n",
			self->dummyType, CCK_THIS_TYPE);
	}
}

static int
cckDummyInit (CckDummy* self, const CckDummyConfig* config)
{

    /* implementation-specific data */
    CckDummyDummyData* data = NULL;

    if(self == NULL) {
	CCK_ERROR("Dummy Dummy init called for an empty dummy\n");
	return -1;
    }

    if(config == NULL) {
	CCK_ERROR("Dummy Dummy init called with empty config\n");
	return -1;
    }

    /* Shutdown will be called on self object anyway, OK to exit without explicitly freeing self */
    CCKCALLOC(self->dummyData, sizeof(CckDummyDummyData));

    data = (CckDummyDummyData*)self->dummyData;

    data->param1 = 42;

    self->param1 = config->param1;

    return 1;
}

static int
cckDummyShutdown (void* component)
{

    if(component == NULL)
	return -1;

    CckDummy* self = (CckDummy*)component;

    if(self->dummyData != NULL) {
	free(self->dummyData);
	self->dummyData = NULL;
    }

    return 1;

}

static CckBool
cckDummyDoSomething (CckDummy* self, int param1)
{

    if(private1(self, &param1)) {

	return CCK_TRUE;

    }

    return CCK_FALSE;

}


static CckBool
cckDummyDoSomethingWithCallback (CckDummy* self, int param1)
{


    if(self->dummyCallback != NULL) {

	self->dummyCallback(CCK_FALSE, &param1);
	return CCK_TRUE;

    }

    return CCK_FALSE;

}

static int
private1(CckDummy* self, int* param1)
{

    self->param1 = 42;
    *param1 = 42;
    return -1;

}


#undef CCK_THIS_TYPE
