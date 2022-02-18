import message_filters


class MessageSubscriber:
    def __init__(self, topics, callback=None):
        self._topics = topics  # [(topic_name, topic_msg), ...]
        self._subscribers = []
        self.msgs = None
        self.callback = callback

    def subscribe(self):
        subscribers = []
        for topic_name, topic_msg in self._topics:
            sub = message_filters.Subscriber(
                topic_name, topic_msg, queue_size=1, buff_size=2 ** 24
            )
            subscribers.append(sub)
        self._subscribers = subscribers

        sync = message_filters.TimeSynchronizer(
            self._subscribers,
            queue_size=100,
        )
        sync.registerCallback(self._callback)

    def unsubscribe(self):
        for sub in self._subscribers:
            sub.unregister()
        self._subscribers = []

    def _callback(self, *msgs):
        self.msgs = msgs
        if self.callback:
            self.callback(*msgs)
