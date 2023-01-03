/*
*  FIFO stack tool
*/

#include "FIFO_stack_tool.h"

int stack_count;
static stack *p;

// initialize new stack
void stack_init( void )
{
	p = (stack *)malloc(sizeof(stack));
	
	stack_count = 0;
	
	p->data = NULL;
	p->prev = NULL; 
	p->next = NULL;	
}

// add data to stack, append new data to end if not empty
void push_stack(void *data)
{
	stack *t = NULL;
	void *d = data;
	
	if(p != NULL)	// check to see if stack was initialized
	{
		if(p->data)	// do we already have data here? if we don't initialize the first
		{
			//if(!p->prev)	// check for null if so allocate space
			t = (stack *)malloc(sizeof(stack));

			t->next = p;
			p->prev = t;
			p = t;
			p->data = d;  // set data for current stack
			p->prev = NULL;
		}else{
			p->data = d;	// if we got here this is the first
			p->next = NULL;
			p->prev = NULL;
		}
	}
}

// pop data off the begining of the stack
void *pop_stack( void )
{
	void *data;
	stack *t = p;
	if(t)
	{
		if(!p)
			data = NULL;
		
		while(t->next) t = t->next;	// iterate till we get to the first in the stack
		
		data = t->data;
		
		if(t->prev)  // check to see if we got more in the FIFO stack
		{
			t->prev->next = NULL;	// set next to null and free current stack
			free(t);
		}else{
			t->data = NULL;	// don't free last stack, set data to null so stack can be used again
			t->next = NULL;
			t->prev = NULL;
		}
		return data;
	}
	return NULL;
}   

