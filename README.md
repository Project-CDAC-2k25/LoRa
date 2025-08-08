# LoRa Based Mesaging Protocol
our Daily Project Journal

As we planning to implement chat-based protocol in STM32 using LoRa, our implementation idea is given in this document.
We inspired these from all knew protocols like Bluetooth, Wi-Fi, and other;

To have basic communication, User will have UI:
1.	For PC’s – a terminal exe file (Uses UART).
2.	For Mobiles – An App type thing (Connected over Bluetooth), which is connected to STM via bluetooth module ( hc-05 ).
User will get a List of available peers/nodes, where he can select a node to chat with. When a node is selected, the stm will create a session by exchanging keys and store it in a structure/array.  Then, User will get interface for chatting, with a textbox at bottom of screen. Message sent are encrypted by the key which is stored in a structure.
On receiving side, receiver will decrypt message and show it to user.

Implementation :
-	We will use two timers ; 
i.e., 1. To send a broadcast signal periodically at random 0-15 seconds with the control field set to 0xFF in packet. 
      2.  For General use , like using for 1 sec ack timer.

The Packet Format we thought of is – 6 to 7 fields;
1.	Source Id 		( 1 Byte )
2.	Receiver Id		( 1 byte )
3.	Control Field ( 1 byte )

|  z  |  y  |  x  |  w  |  v  |  u  |  t  |  s  |

z - Indicates whether the actual message is frangmenteed or not.

Upper Nibble for Frame relatesd info and lower nibble ( i.e., bits v ,u , t, s ) are for packet type identification.

Lower nibble type :
-	00  for Data
-	01 for Ack
-	02 for NACK
-	03 for Connect req ( start key exchange between them by sending its public key in data field)
-	04 for connect ack
-	F  for Broadcast
4.	Frame Num    ( 1 byte )
 	upper Nibble is for describing total number of Frames to sent.
 	Lower Nibble is for current frame's number.
5.  MsgId/SeqId // for uniquely identify the frame is of same message or not. Much usefull in framing time...
6.	Packet Len ( Data )
7.	Data [ 100 ]
8.	CRC  ( sbytes  )
This is the format with which the data between lora to lora is transferred; 


as we are gave a nibble for indicating the length of frame , the limit of creating frames is  of 16. it means we can make chuncks for 16*100 = 1600 bytes of data at MAx. It is more than enough for now. ( I think ).


For communication between the interface and STM , it uses UART( PC) or Bluetooth ( Mobiles) usually the data is sent as in string format so , we thought to format and send it as in form :

    “TYPE: req/send;	DATA: PeerList/Msg;	ID:<any ascii letter- Node Id>;”

For Example :

If  User requests a Peerlist , the string is sent is

    TYPE:REQ;	DATA:PeerList;

If user wants to send data/msg . we should send it as 
	
     TYPE:SEND; DATA:<MSG TO SEND>; ID:<USER NODE ID>;
 
If user wants to start a convo with a node
	
     TYPE:CONN;ID:<NODE ID>;		//This will start stm to make a connection request. ( key exchanges)

 	
