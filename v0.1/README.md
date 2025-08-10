19.8.25


We added send queue for enqueueing packets which need to  be transmitted after byte stuffed framing.

Considering the frame format, as our mentor suggested we added PPP type byte stuffing to distinguish the start and end of the frame. 
implemented 0x7E as start and end flag and if 0x7E present in data , the logic makes that byte into two bytes. 
i.e., a escape sequence 0x7D is added following by original byte XOR'ed with 0x20. 
On receiveing side the opposite is done.

As of now, we are just focussed on unfragmented frames.

