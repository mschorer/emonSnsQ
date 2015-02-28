//
// mqueue
//

typedef struct {
  int voltage;
  int currP1;
  int currP2;
  int currP3;
  int currP4;
} mQueueEntry;

typedef struct {
  byte head;
  byte tail;
  mQueueEntry queue[ QUEUE_LENGTH];
} mQueue;

byte mqIsEmpty( mQueue* mq) {
  return ( mq->head == mq->tail);
}

mQueueEntry* mqFirst( mQueue* mq) {
  mQueueEntry* hd = &mq->queue[ mq->head];

  return hd;
}

mQueueEntry* mqEnqueue( mQueue* mq) {
  mq->head = ++mq->head % QUEUE_LENGTH;
  if ( mqIsEmpty( mq)) mq->tail = ++mq->tail % QUEUE_LENGTH;
  
  return &mq->queue[ mq->head];
}

mQueueEntry* mqLast( mQueue* mq) {
  mQueueEntry* tail = NULL;

  if ( ! mqIsEmpty( mq)) {
    tail = &mq->queue[ mq->tail];
    mq->tail = ++mq->tail % QUEUE_LENGTH;
  }

  return tail;
}
