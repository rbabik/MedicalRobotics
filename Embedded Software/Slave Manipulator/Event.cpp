/* Includes ------------------------------------------------------------------*/
#include "Event.h"
#include "state_machine.h"

/* Macros ------------------------------------------------------------------*/


/* Module variables ---------------------------------------------------------*/
//instantiate a variable of queue struct type
ev_queue event_queue; 

/* functions ------------------------------------------------------------------*/

//initialize the event queue
void initQueue(void)
{
    //set size of queue to 0
    event_queue.queueSize = 0;
    //set start pointer to 0
    event_queue.queueStart = 0;
    //set end pointer to 0
    event_queue.queueEnd = 0;
}

//function to add event to queue********
void addToQueue(event_type new_event)
{    
    //check if queue is not empty -- if it is, then cant move pointer to end of it
    if (event_queue.queueSize > 0)
    {
        //move end of queue pointer to one plus the previous end
        event_queue.queueEnd = event_queue.queueEnd + 1;
    }
    //add one to the queue size
    event_queue.queueSize = event_queue.queueSize + 1;
    
    //check if the end of queue is at the max queue size
    if (event_queue.queueSize >= MAX_EVENT_QUEUE_SIZE)
    {
        //if it is, set the end of queue pointer to zero spot
        event_queue.queueEnd = 0;
    }
    //set the end of the queue in the queue struct queue to be the added event
    event_queue.queue[event_queue.queueEnd] = new_event;
}

//function to service the event at the front of the queue**********
void serviceEventQueue(void)
{
    //create enum event type for current event
    event_type current_event;

    //if queue is not empty
    if (event_queue.queueSize > 0)
    {
        //set the current event equal to the event at start of queue
        current_event = event_queue.queue[event_queue.queueStart];
        //delete that event we just grabbed from the queue
        removeFromQueue();
        //pass the current event to the state machine
        stateMachine(current_event);
    }
}   

//function to remove event from queue*********
void removeFromQueue(void)
{
    //check if the queue size is not empty
    if (event_queue.queueSize > 0)
    {
        //check if queue size is greater than 1 (so won't be empty after a delete)
        if (event_queue.queueSize > 1)
        {
            //point start of queue to advance one forward
            event_queue.queueStart = event_queue.queueStart + 1;        
        }
        //decrement queue size by 1
        event_queue.queueSize = event_queue.queueSize - 1;
        //check if start of queue is at max queue size
        if (event_queue.queueStart >= MAX_EVENT_QUEUE_SIZE)
        {
            //move start of queue pointer to element zero
            event_queue.queueStart = 0;
        }
    }
}  
    
    
        
