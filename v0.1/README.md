10.8.25


We added send queue for enqueueing packets which need to  be transmitted after byte stuffed framing.

Considering the frame format, as our mentor suggested we added PPP type byte stuffing to distinguish the start and end of the frame. 
implemented 0x7E as start and end flag and if 0x7E present in data , the logic makes that byte into two bytes. 
i.e., a escape sequence 0x7D is added following by original byte XOR'ed with 0x20. 
On receiveing side the opposite is done.

As of now, we are just focussed on unfragmented frames.

11.8.25


We added uart commands and other related functions with UART which used in communication between STM and UI ( APPP ).
All commands are terminated with souble semicolon ;; . 

Upon search , found a software which can be used to debug our UI python file. Found that we can create a pair of virtual ports,  and test the py file with our UART serial commands.

12.8.25

Mock Interview Day , Did Nothing. 
Thought to Debug UI's exe file but its became late when i reached my place, Tired and postponed to Next day.

13.8.25 

Iinstall the com0com by sourceforge; Link is given below.
Facing Driver issue and other. If i can manage this by today , i can debug py file like how its responding to our UART commands and so on.

https://sourceforge.net/projects/com0com/files/com0com/
com0com is not installing properly , so searched for an alternatives and found other one and used it.

14.8.25

Added debug console directly into the py app , such that there will be less serial com apps like hercules and see all the printf ( serial messages ) in that console;

15.8.25

Finally, tried an bare skelton model of this. 
after trying we getinng messages and ack back and are well according to logic we written in stm.
we got message from other message from user in my screen but next the program strucked and dont know whwt happen and also wanted to reset controller every time , and after few tries the messages didnt even came to me;
 
somewhere felt like we need to implment receive buffer/queue too for receving . it might be the one of the reason for it;

but i thought our implemwntation might be slow due to super main loop implementation, instead of RTOS. and i read the difference befeore but felt it live today .
we predicted it atfirst itself, but we wanna do it purely on stm supermain loop and make it efficient; will seee what happens !

