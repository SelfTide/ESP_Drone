#ifndef FIFO_STACK_TOOL
#define FIFO_STACK_TOOL

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


typedef struct stack stack;

struct stack{
	void *data;
	stack *prev, *next;
};

extern int stack_count;

void stack_init( void );
void push_stack(void *data);
void *pop_stack( void );

#endif