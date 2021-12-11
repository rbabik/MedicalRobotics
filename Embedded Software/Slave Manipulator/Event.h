/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EVENT_H_
#define __EVENT_H_


/* Includes ------------------------------------------------------------------*/

//define macro for max queue size
#define MAX_EVENT_QUEUE_SIZE 20


/*Instantiate---------------------------------------------------------*/

//create enum for the different events that can occur (that we care about)
typedef enum
{
    NO_EVENT,
    START_BUTTON_PRESSED,
    START_BUTTON_UNPRESSED,
    SHUT_DOWN_COMPLETE,
    ENCODER_INIT_BUTTON_PRESSED,
    FINE_CONTROL_BUTTON_PRESSED,
    FINE_CONTROL_BUTTON_UNPRESSED,
    MOVEMENT_STARTED,
} event_type;

//create a struct to hold the specific event 
typedef struct ev_queue
{
    //create queue
    event_type queue[MAX_EVENT_QUEUE_SIZE];
    //size of queue variable
    int queueSize;
    //start of queue variable
    int queueStart;
    //end of queue variable
    int queueEnd;
} ev_queue;

/*Function definitions---------------------------------------------------------*/

//initialize the event queue
void initQueue(void);

//function to add event to queue
void addToQueue(event_type new_event);

//function to service the event at the front of the queue
void removeFromQueue(void);

//function to remove event from queue
void serviceEventQueue(void);


#endif /*__EVENT_H_ */
