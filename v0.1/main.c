/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include "LoRa.h"
#include "string.h"
#include <stdint.h>
#include "byte_stuffing.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define RECENT_IDS_PER_SRC 4
typedef struct {
	uint8_t Node_ID;
	uint8_t RSSI;
	uint32_t lastseen;
	uint8_t ids[RECENT_IDS_PER_SRC];
	uint8_t nextMsgId;                    // NEXT msg id to use when sending to this peer
} NodeList;


typedef struct  __attribute__((packed)){
	uint8_t sourId;
	uint8_t destId;
	// 7th bit - to indicate fframe fragmented
	// 6,5,4 - Number of Frames
	// 3,2,1,0 - 0 - Data, 1 - ACK, 2 - NACK, 3 - CONN , F- Broadcast
	uint8_t control;
	uint8_t frameNo;
	uint8_t msgId;
	uint8_t pLen; // Length of actual data
	uint8_t data[100];
	uint16_t crc16;  // CRC checksum added at the end
} Packet;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define THIS_NODE 'A'
// Control field bits
#define CTL_TYPE_MASK 0x0F
#define CTL_TYPE_DATA 0x00
#define CTL_TYPE_ACK 0x01
#define CTL_TYPE_NACK 0x02
#define CTL_TYPE_CONN 0x03

#define CTL_FRAG_FLAG 0x80
#define CTL_WIN_SHIFT 4
#define CTL_WIN_MASK 0x70
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
LoRa myLoRa;
struct NodeManager {
	uint8_t NoofNodesPresent;
	NodeList Nodes[10];	//Can handle only ten peers

}nm={0};
//uint8_t rx_buf[150] = {0};
uint8_t packetReceivedFlag,sendACK,sendBcast,rlen;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
static inline uint8_t control_get_type(uint8_t c) { return c & CTL_TYPE_MASK; }
static inline uint8_t control_get_win(uint8_t c) { return (c & CTL_WIN_MASK) >> CTL_WIN_SHIFT; }
static inline uint8_t control_get_frag(uint8_t c) { return (c & CTL_FRAG_FLAG) ? 1 : 0; }
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
int __io_putchar(uint8_t ch)
#else
int fputc(int ch,FILE *fp)
#endif
{

	HAL_UART_Transmit(&huart2,&ch, 1, HAL_MAX_DELAY);
	return ch;

}

Packet newPacket(){
	Packet test = {THIS_NODE,0 };
	return test;
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	//Timer Callbacks;
	if(htim==&htim4){
		// TIM4 for ack and general purpose

	}else if(&htim3==htim){
		sendBcast=1;
	}
}

uint8_t validate_packet(Packet *recv){
	if((recv->destId==THIS_NODE)||(recv->destId==0xFF))	// FF mean its a Broadcast
		return 0;
	return 1;
}

uint16_t crc16_ccitt(const uint8_t* data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)(data[i]) << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

static inline uint8_t make_control(uint8_t fragFlag, uint8_t win, uint8_t type) {
// win is 0..7
return (fragFlag ? CTL_FRAG_FLAG : 0) | ((win & 0x07) << CTL_WIN_SHIFT) | (type & CTL_TYPE_MASK);
}


uint8_t serializePacket(const Packet* pkt, uint8_t* buf, size_t bufsize)
{
    if (!pkt || !buf)
    	return 1;

    uint16_t total_length = 6 + pkt->pLen + 2; // header (5) + data + crc (2)

    if (bufsize < total_length)
    	return 1;

    buf[0] = pkt->sourId;
    buf[1] = pkt->destId;
    buf[2] = pkt->control;
    buf[3] = pkt->frameNo;
    buf[4] = pkt->msgId;
    buf[5] = pkt->pLen;
    memcpy(&buf[6], pkt->data, pkt->pLen);

    // Calculate CRC over all fields except the CRC itself
    uint16_t crc = crc16_ccitt(buf, 6 + pkt->pLen);
    buf[6 + pkt->pLen] = (crc >> 8) & 0xFF;    // CRC high byte
    buf[7 + pkt->pLen] = crc & 0xFF;           // CRC low byte

    return total_length; // total bytes to send
}

uint8_t deserializePacket(const uint8_t* buf, size_t len, Packet* pkt)
{

    if (!buf || !pkt || len < 8) return 1;  // minimum packet size chcking

    uint8_t pLen = buf[5];
    if (pLen > 100 || (len != (size_t)(pLen + 8)))
    	return 1; // size mismatch

    // Verify CRC
    uint16_t received_crc = ((uint16_t)buf[6 + pLen] << 8) | buf[7 + pLen];
    uint16_t calc_crc = crc16_ccitt(buf, 6 + pLen);
    if (received_crc != calc_crc)
        return 1; // CRC check failed

    pkt->sourId = buf[0];
    pkt->destId = buf[1];
    pkt->control = buf[2];
    pkt->frameNo = buf[3];
    pkt->msgId	=  buf[4];
    pkt->pLen = pLen;
    memcpy(pkt->data, &buf[6], pLen);
    pkt->crc16 = received_crc;

    return 0; // success
}



static uint8_t tx_buffer[256];
static size_t tx_index;

static void send_byte_to_txbuf(uint8_t byte) {
    if (tx_index < sizeof(tx_buffer)) {
        tx_buffer[tx_index++] = byte;
    }
}

// Transmit full frame with byte stuffing and flags.
int transmit_frame_with_byte_stuffing(uint8_t* data, size_t length) {
    tx_index = 0;
    send_byte_to_txbuf(0x7E);  // Start flag
    for (size_t i = 0; i < length; i++) {
        transmit_byte_stuffed(data[i], send_byte_to_txbuf);
    }
    send_byte_to_txbuf(0x7E);  // End flag

    // Use your LoRa transmit function (timeout 1000 ms)
    return LoRa_transmit(&myLoRa, tx_buffer, tx_index, 1000);
}

#define P_SIZE 150
#define TXQ_SIZE 8
#define TX_RETRIES 5
#define LOCAL_WINDOW 3
#define TX_RETRY_MAX 4
#define TX_ACK_TIMEOUT_MS 3000

typedef struct {
Packet pkt;
uint8_t len;
uint8_t ser[P_SIZE];
uint8_t in_use;
uint8_t sent;
uint8_t acked;
uint8_t retries;
uint32_t t_send;
} TxEntry;
static TxEntry txq[TXQ_SIZE];
static uint8_t txq_head, txq_tail=0;
static int txq_is_empty(void){
	return ( txq_head == txq_tail);
}
static int txq_is_full(void){
	return ((txq_head+1)%TXQ_SIZE == txq_tail);
}

/* We should make sure our TX path always knows the node beforehand (via nm_duplicate_and_update() in RX),
 * unless we explicitly want to auto-add unknown dests.*/
uint8_t alloc_msgid(uint8_t destId)
{
    if (destId == 0xFF) {
        return 0; // broadcast always 0
    }
    // Just find, update, and return — node must already exist
    for (uint8_t i = 0; i < nm.NoofNodesPresent; i++) {
        if (nm.Nodes[i].Node_ID == destId) {
            uint8_t id = nm.Nodes[i].nextMsgId;
            nm.Nodes[i].nextMsgId = (id + 1) & 0xFF; // wrap at 255 -> 0
            return id;
        }
    }
    // Should never happen if NodeManager is updated before TX
    printf("alloc_msgid: dest %c not in NodeManager!\n", destId);
    return 0;
}

int txq_enque_unfrag(uint8_t destId, uint8_t * payload, uint8_t plen){

	if(txq_is_full()){
		printf("TXQ is full, plz wait\r\n");
		return -1;
	}
	TxEntry* temp=&txq[txq_head];
	//Packet pkt= {0};
	memset(temp,0,sizeof(*temp));
	temp->pkt.sourId  = THIS_NODE;
	temp->pkt.destId  = destId;
	temp->pkt.msgId   = alloc_msgid(destId); // <-- new per-node allocation
	temp->pkt.control = make_control(0, LOCAL_WINDOW, CTL_TYPE_DATA);
	temp->pkt.frameNo = 0x00;
	temp->pkt.pLen    = plen;
	memcpy(temp->pkt.data, payload, plen);

	temp->len = serializePacket(&temp->pkt, temp->ser, sizeof(temp->ser));

	if (temp->len == 1) {
		printf("Serialization failed\n");
		return -2;
	}

	temp->in_use = 1;
	txq_head = (txq_head + 1) % TXQ_SIZE;
	return 0;
}

int txq_enque_unfrag_with_msgid(uint8_t destId, uint8_t *payload, uint8_t plen, uint8_t msgid) {
    if(txq_is_full()) {
        printf("TXQ is full, plz wait\r\n");
        return -1;
    }
    TxEntry* temp = &txq[txq_head];
    memset(temp,0,sizeof(*temp));

    temp->pkt.sourId = THIS_NODE;
    temp->pkt.destId = destId;
    temp->pkt.msgId  = msgid; // use provided ID
    temp->pkt.control= make_control(0, LOCAL_WINDOW, CTL_TYPE_DATA);
    temp->pkt.frameNo= 0x00;
    temp->pkt.pLen   = plen;
    memcpy(temp->pkt.data, payload, plen);

    temp->len = serializePacket(&temp->pkt, temp->ser, sizeof(temp->ser));
    if (temp->len == 1) {
        printf("Serialization failed\n");
        return -2;
    }
    temp->in_use = 1;
    txq_head = (txq_head + 1) % TXQ_SIZE;
    return 0;
}


static int txq_outstanding(){
	int cnt=0;
	for(int i=txq_tail; i!=txq_head; i=(i+1)%TXQ_SIZE){
		if(txq[i].in_use && txq[i].sent && txq[i].acked )
			cnt++;
	}

	return cnt;
}

void txq_try_send(void){
	int out = txq_outstanding();
	if(out>LOCAL_WINDOW)
		return;

	// We should send the entries of txqueue , if entry is unsent and
	//also making sure that live sent and unacked packets are less than local_window

	for(int i=txq_tail; i!=txq_head && out < LOCAL_WINDOW;	 i=(i+1)%TXQ_SIZE){
		TxEntry* e = &txq[i];
		if(!e->in_use || e->sent)
			continue;

		if(transmit_frame_with_byte_stuffing(e->ser, e->len)){
			e->sent=1;
			e->t_send=HAL_GetTick();
			e->retries=0;
			out++;
			printf("Tx'ed msg of Id %d to %c ; len =%d\r\n",e->pkt.msgId,e->pkt.destId,e->pkt.pLen);
		}else{
			printf("meg tx failed \r\n");
			break;
		}


	}


}

void txq_check_retransmit(void)
{
	uint32_t now = HAL_GetTick();

	//iterating through complte loop
	for (int i=txq_tail; i!=txq_head; i=(i+1)%TXQ_SIZE) {
		TxEntry* e = &txq[i];
		if (!e->in_use || !e->sent || e->acked)
			continue;	//we skip entries which are unused, unsent, ack'ed

		if ((now - e->t_send) >= TX_ACK_TIMEOUT_MS) {
			if (e->retries >= TX_RETRY_MAX) {
				printf("TX drop msgId=%u dest=%c after retries\n", e->pkt.msgId, e->pkt.destId);
				// drop this entry
				e->in_use=0;
				if (i == txq_tail) {
					while (!txq_is_empty() && !txq[txq_tail].in_use)
						txq_tail = (txq_tail+1) % TXQ_SIZE;
				}
				continue;
			}


			//retransmit
			if (transmit_frame_with_byte_stuffing(e->ser, e->len)) {
					e->t_send = now;
					e->retries++;
					printf("Retransmit msgId=%u try=%u\n", e->pkt.msgId, e->retries);
				}
		}
	}

	// pop acked entries from head of queue
	while (!txq_is_empty() && txq[txq_tail].in_use && txq[txq_tail].acked) {
		txq[txq_tail].in_use = 0;
		txq_tail = (txq_tail+1) % TXQ_SIZE;
	}



}

void txq_mark_acked(uint8_t from, uint8_t to, uint8_t msgId)
{
// from = ACK sender (original receiver), to = our node
	for (int i=txq_tail; i!=txq_head; i=(i+1)%TXQ_SIZE) {
		TxEntry* e = &txq[i];
		if (!e->in_use)
			continue;	//skip for the entries which are not in use
		if (e->pkt.destId == from && e->pkt.sourId == to && e->pkt.msgId == msgId) {
			e->acked = 1;
			printf("ACKed msgId=%u by %c\n", msgId, from);

			return;
		}
	}
	// Not found (stale ACK or already removed)
}

void uart_send_ack(const char* nodeId, uint8_t msgid)
{
    printf("TYPE:ACK;ID:%s;MSGID:%u;;\r\n", nodeId, msgid);
}
void uart_send_peerlastseen(void) {
    printf("TYPE:PEERLIST;DATA:");
    for(uint8_t i=0;i<nm.NoofNodesPresent;i++) {
        printf("%c(lastseen:%lu's)%s", nm.Nodes[i].Node_ID,
            (HAL_GetTick() - nm.Nodes[i].lastseen)/1000,
            (i!=nm.NoofNodesPresent-1)?",":"");
    }
    printf(";;\r\n");
}

void uart_send_update(const char* nodeId, const char* msg) {
    printf("TYPE:UPDATE;ID:%s;MSG:%s;;", nodeId, msg);
}
void uart_send_peerlist(void) {
    printf("TYPE:PEERLIST;DATA:");
    for(uint8_t i=0;i<nm.NoofNodesPresent;i++)
        printf("%c%s", nm.Nodes[i].Node_ID, (i!=nm.NoofNodesPresent-1)?",":"");
    printf(";;");
}
void uart_send_notif(const char* msg) {
    printf("TYPE:NOTIF;MSG:%s;;\r\n", msg);
}


int is_broadcast(uint8_t destId) {
	return destId == 0xFF;
}

int send_broadcast(const uint8_t *payload, uint8_t plen)
{
    Packet pkt={0};
    pkt.sourId  = THIS_NODE;
    pkt.destId  = 0xFF;
    pkt.msgId   = 0; // broadcast fixed ID
    pkt.control = make_control(0, LOCAL_WINDOW, CTL_TYPE_DATA);
    pkt.frameNo = 0;
    pkt.pLen    = plen;
    memcpy(pkt.data, payload, plen);

    uint8_t buf[50];
    uint8_t len = serializePacket(&pkt, buf, sizeof(buf));
    if (len != 1) {
        return transmit_frame_with_byte_stuffing(buf, len);
    }
    return 0;	//unsuccessful
}


void send_ack(uint8_t to, uint8_t msgId)
{
	Packet ack = {0};
	ack.sourId = THIS_NODE;
	ack.destId = to;
	ack.msgId = msgId;
	ack.control= make_control(0, LOCAL_WINDOW, CTL_TYPE_ACK);
	ack.frameNo= 0;
	ack.pLen = 0;


	uint8_t ser[50];
	uint8_t len = serializePacket(&ack, ser, sizeof(ser));
	if (len == 1)
		return;
	transmit_frame_with_byte_stuffing(ser, len);
}


#define RECENT_IDS_PER_SRC 4

// Returns 0 = new message (process payload),
//	1 = duplicate message from known node (suppress),
//	2 = unknown unicast (ignore completely), unknown node (never added)
// 	isBroadcast=1 means this packet is from a broadcast sender
uint8_t nm_duplicate_and_update(uint8_t sourId, uint8_t msgId, int rssi, uint8_t isBroadcast)
{
    uint32_t now = HAL_GetTick();
    // Search for existing node
    for (uint8_t i = 0; i < nm.NoofNodesPresent; i++) {
        if (nm.Nodes[i].Node_ID == sourId) {

            nm.Nodes[i].lastseen = HAL_GetTick();
            nm.Nodes[i].RSSI = rssi;
            printf("Updated node %c lastseen to %lu\n", sourId, (unsigned long)nm.Nodes[i].lastseen);
            // --- Duplicate check ---
            for (uint8_t j = 0; j < RECENT_IDS_PER_SRC; j++) {
                if (nm.Nodes[i].ids[j] == msgId) {
                    return 1; // Duplicate
                }
            }

            // Shift ids down and insert this one at [0]
            for (int j = RECENT_IDS_PER_SRC - 1; j > 0; j--) {
                nm.Nodes[i].ids[j] = nm.Nodes[i].ids[j - 1];
            }
            nm.Nodes[i].ids[0] = msgId;

            return 0; // New message
        }
    }

    // --- Node not found ---
    // Only add if this was a broadcast discovery
    if (isBroadcast) {
    if (nm.NoofNodesPresent < 10) {
        uint8_t idx = nm.NoofNodesPresent++;
        nm.Nodes[idx].Node_ID = sourId;
        nm.Nodes[idx].RSSI = rssi;
        nm.Nodes[idx].lastseen = now;
        memset(nm.Nodes[idx].ids, 0xFF, sizeof(nm.Nodes[idx].ids)); // init to invalid
        nm.Nodes[idx].ids[0] = msgId;
        printf("New node discovered via broadcast: %c\n", sourId);
        return 0; // Treated as new message
        }else {
            printf("Node table full! Cannot add %c\n", sourId);
        return 1; // Treat as duplicate to avoid processing
        }
    }
    // If not broadcast and unknown → ignore completely
      printf("Ignoring unicast from unknown node %c\n", sourId);
      return 2; // Treat as duplicate so payload won't be processed
  }


void handle_unfrag_data_packet(const Packet* p)
{
//	Drop if not for me nor broadcast
//	if (!(p->destId == THIS_NODE || is_broadcast(p->destId))) return;
	const uint8_t isBcast = is_broadcast(p->destId) ? 1 : 0;
	uint8_t dup = nm_duplicate_and_update(p->sourId, p->msgId, LoRa_getRSSI(&myLoRa), isBcast);
	char idstr[2] = { p->sourId, '\0' };
	if (dup == 2) {
        // Unicast unknown —ignore
        return;
    }
    // For broadcast: process but do not ACK to avoid storms
    if (isBcast) {
		if (!dup) {
			 // Data from a new or known node’s broadcast
			printf("BCAST %c: %.*s RSSI=%d\n", p->sourId, p->pLen, (char*)p->data, LoRa_getRSSI(&myLoRa));
		}
		else {
			// Duplicate broadcast: known node, already seen msgId, ignore
			printf("Duplicate broadcast from %c msgId=%u ignored\n", p->sourId, p->msgId);
		}
	return;
	}
    // Unicast to me, only from known sender
	if (!dup) {
		 // Brand new msg from known node
		printf("DATA from %c: %.*s RSSI=%d\n", p->sourId, p->pLen, (char*)p->data, LoRa_getRSSI(&myLoRa));
		// Relay to UI
		uart_send_update(idstr, (char*)p->data);
		// Process data as needed...
	} else {
		// Duplicate unicast (retransmit tolerance/ack drop handling)
		printf("Duplicate DATA %s from %c msgId=%u\n",(char*)p->data, p->sourId, p->msgId);

		uart_send_update(idstr, (char*)p->data);
	}
	send_ack(p->sourId, p->msgId);	// Always ACK unicast, including duplicates
}

void on_packet_received(const Packet* p)
{
	uint8_t frag = control_get_frag(p->control);
	uint8_t type = control_get_type(p->control);
	if (frag) {
		 // Fragmented path to implement next phase (with bitmap selective ACK)

	}
	// Unfragmented path first
		switch(type) {
		case CTL_TYPE_DATA:
			handle_unfrag_data_packet(p);
			break;
		case CTL_TYPE_ACK:
			// ACKs always unfragmented
			txq_mark_acked(p->sourId, p->destId, p->msgId);
			// Notify UI over UART
			char idstr[2] = {p->sourId, 0};
			uart_send_ack(idstr, p->msgId);
			break;
		case CTL_TYPE_NACK:
        // Optional: trigger immediate retransmit or mark for retry
			break;
		case CTL_TYPE_CONN:
			break;
		default :
			break;
		}
}


#define RX_UNSTUFFED_MAX_LEN 150

uint8_t rx_buf[RX_UNSTUFFED_MAX_LEN];
ByteUnstuffState unstuff_state = {0};

typedef enum {
    WAIT_FOR_START,
    RECEIVING_FRAME
} rx_state_t;

static rx_state_t rx_state = WAIT_FOR_START;
static size_t rx_unstuffed_len = 0;

void process_incoming_byte(uint8_t byte) {
    switch(rx_state) {
        case WAIT_FOR_START:
            if(byte == 0x7E) {
                rx_unstuffed_len = 0;
                unstuff_state.escape_flag = 0;
                rx_state = RECEIVING_FRAME;
            }
            break;
        case RECEIVING_FRAME:
            if(byte == 0x7E) { // End of frame
                if(rx_unstuffed_len >= 8) { // Minimal header + CRC length
                    Packet pkt;
                    if(deserializePacket(rx_buf, rx_unstuffed_len, &pkt) == 0
                       && validate_packet(&pkt) == 0) {
                        on_packet_received(&pkt);
                    } else {
                        // CRC or deserialize failed
                        printf("Packet CRC or deserialize error\r\n");
                    }
                }else {
                    printf("Frame too short, discarded\r\n");
                }
                rx_state = WAIT_FOR_START;
            } else {
                uint8_t unstuffed;
                if(receive_byte_unstuffed(byte, &unstuffed, &unstuff_state)) {
                    if(rx_unstuffed_len < RX_UNSTUFFED_MAX_LEN) {
                        rx_buf[rx_unstuffed_len++] = unstuffed;
                    } else {
                        // buffer overflow, reset state
                        rx_state = WAIT_FOR_START;
                        printf("RX buffer overflow\r\n");
                    }
                }
            }
            break;
    }
}



void prune_stale_nodes(uint32_t stale_ms)
{
    uint32_t now = HAL_GetTick();
    uint8_t i = 0;

    while (i < nm.NoofNodesPresent) {
        if ((now - nm.Nodes[i].lastseen) > stale_ms) {
            //printf("Node %c timed out, removing from list!\n", nm.Nodes[i].Node_ID);
        	printf("Node %c removed (last seen %lu ms ago)\n",
        			nm.Nodes[i].Node_ID, (unsigned long)(now - nm.Nodes[i].lastseen));
        	// Shift later nodes down
            for (uint8_t j = i+1; j < nm.NoofNodesPresent; j++)
                nm.Nodes[j-1] = nm.Nodes[j];
            nm.NoofNodesPresent--;
            // Do not increment i so we re-examine the moved node
        } else {
            i++;
        }
    }
}

// Example: call every 10 sec in main loop
//prune_stale_nodes(60000); // 1 min timeout

void nm_print_nodes(void)
{
    printf("\r\n--- NodeManager: %u active node(s) ---\r\n", nm.NoofNodesPresent);
    //uint32_t now = HAL_GetTick();

    for (uint8_t i = 0; i < nm.NoofNodesPresent; i++) {
        NodeList *n = &nm.Nodes[i];
        uint32_t age_ms = HAL_GetTick() - n->lastseen;

        printf("Node %c | RSSI: %d dBm | last seen: %lu ms ago | recentMsgIds: ",
               n->Node_ID, n->RSSI, (unsigned long)age_ms);

        // Print recent msgIds history
        for (int j = 0; j < RECENT_IDS_PER_SRC; j++) {
            if (n->ids[j] == 0xFF) printf("-- ");
            else                   printf("%u ", n->ids[j]);
        }
        printf("\r\n");
    }
    printf("-------------------------------\r\n");
}

//#define RX_BUF_SIZE 256
//uint8_t uart_rx_buffer[RX_BUF_SIZE];
#define UART_CMD_BUF_SIZE 256
uint8_t uart_rx_dma_buf[UART_CMD_BUF_SIZE]; // DMA buffer
char uart_parse_buf[UART_CMD_BUF_SIZE];     // For processing
volatile uint16_t uart_parse_len = 0;

typedef struct {
    char type[10];   // always present
    char data[150];  // optional
    char id[10];     // often optional, depends on type
    int  msgid;         // parsed from MSGID field, default -1 if not given
} UARTCmd;

void handle_uart_cmd(const UARTCmd* ucmd) {
	if (strcmp(ucmd->type, "SEND") == 0) {
		if (ucmd->id[0] == '\0') {
			printf("SEND command missing ID\n");
			return;
		}
		// enqueue message to ID
		uint8_t dest = ucmd->id[0];
		uint8_t msglen = strlen(ucmd->data);
		int ret;
		if(ucmd->msgid >=0){
			//send with provided msg Id
			ret = txq_enque_unfrag_with_msgid(dest, (uint8_t*)ucmd->data, msglen, (uint8_t)ucmd->msgid);
		} else {
		// fallback: auto-allocate msgId
		ret =  txq_enque_unfrag(dest, (uint8_t*)ucmd->data, msglen);
	   }
		if (ret == 0) uart_send_notif("Queued");
		        else uart_send_notif("QueueFull");




	}
	else if (strcmp(ucmd->type, "REQ") == 0) {
		// ID not required when asking for PeerList etc
		if (strcmp(ucmd->data,"PeerList")==0){
			 uart_send_peerlist();
		}
	}
	else if (strcmp(ucmd->type, "CONN") == 0) {
		if (ucmd->id[0] == '\0') {
			printf("CONN command missing ID\n");
			return;
		}
		// start pairing
	}
	// etc...
}


void uart_parse_command(char *cmd, uint16_t length) {
    UARTCmd ucmd = {0};
    ucmd.msgid = -1; // default if not supplied

    char *token = strtok(cmd, ";");  // split by semicolon

    while (token) {
        if (strncmp(token, "TYPE:", 5) == 0) {
            strncpy(ucmd.type, token + 5, sizeof(ucmd.type) - 1);
        }
        else if (strncmp(token, "DATA:", 5) == 0) {
            strncpy(ucmd.data, token + 5, sizeof(ucmd.data) - 1);
        }
        else if (strncmp(token, "ID:", 3) == 0) {
            strncpy(ucmd.id, token + 3, sizeof(ucmd.id) - 1);
        }else if (strncmp(token, "MSGID:", 6) == 0) {
        	 ucmd.msgid = atoi(token + 6);   // convert to integer
        }
        // easy to add else if for STATUS:, TIME: etc later
        token = strtok(NULL, ";"); // next field
    }

    // Validate TYPE mandatory
    if (ucmd.type[0] == '\0') {
        printf("Invalid command: TYPE missing\n");
        return;
    }

    handle_uart_cmd(&ucmd);
}

void start_uart_rx_dma(void) {
	// Start UART DMA RX with Idle detection
    if(HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart_rx_dma_buf, UART_CMD_BUF_SIZE)!= HAL_OK) {
        Error_Handler();
    }
    // Assuming hdma_usart2_rx is your DMA handle for UART RX
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    //__HAL_DMA_DISABLE_IT(huart2.hdmatx, DMA_IT_HT);

}
uint8_t uart_parse;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if(huart->Instance == USART2) {
    	 if (Size && Size < UART_CMD_BUF_SIZE) {
    	        memcpy(uart_parse_buf, uart_rx_dma_buf, Size);
    	        uart_parse_buf[Size] = 0; // Ensure null-termination
    	        uart_parse_len = Size;
    	        uart_parse=1;
    	    }
        // Restart reception for next frame
    	 HAL_UART_DMAStop(huart);
    	 start_uart_rx_dma();
    }
}

#define RX_RING_SIZE 6
#define RX_BUF_MAXLEN 150
typedef struct {
    uint8_t data[RX_BUF_MAXLEN];
    size_t len;
} RxBufEntry;
RxBufEntry rx_ring[RX_RING_SIZE];
uint8_t rx_head = 0;
uint8_t rx_tail = 0;
uint8_t rx_count = 0;
uint8_t rx_overflow = 0; // Optional, for diagnostics


void process_received_data(){
	Packet rxpckt={0};
	if(deserializePacket(rx_buf, rlen, &rxpckt) || validate_packet(&rxpckt)){
		printf("Error occured in Deserializing...\r\n");
		return;
	on_packet_received(&rxpckt);

	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin != GPIO_PIN_1)
		return;
	// Receive the Data;;;; - here our rx_buf as max as possible , because receive func will do take
	uint8_t buf[RX_BUF_MAXLEN];
	uint8_t rlen = LoRa_receive(&myLoRa, buf, sizeof(buf));
	if (rx_count < RX_RING_SIZE) {
		memcpy(rx_ring[rx_head].data, buf, rlen);
		rx_ring[rx_head].len = rlen;
		rx_head = (rx_head + 1) % RX_RING_SIZE;
		rx_count++;
	} else {
		rx_overflow = 1; // Diagnostic
		// You can print a warning here if you want
	}
}
void process_rx_ring(void) {
    while (rx_count > 0) {
        size_t len = rx_ring[rx_tail].len;
        uint8_t *data = rx_ring[rx_tail].data;
        // Directly call your packet/frame handling per received packet
        // If you destuff/frame one byte at a time:
        for (size_t i = 0; i < len; i++) {
            process_incoming_byte(data[i]);
        }
        // If you want to do everything at once and you have a function for it, call e.g.
        // process_entire_packet(data, len);

        rx_tail = (rx_tail + 1) % RX_RING_SIZE;
        rx_count--;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  start_uart_rx_dma();

  /* USER CODE BEGIN 2 */
  myLoRa = newLoRa();
// Setup LoRa pins & SPI handler specific to your board!
  myLoRa.CS_port = GPIOB;
  myLoRa.CS_pin = GPIO_PIN_6;
  myLoRa.reset_port = GPIOA;
  myLoRa.reset_pin = GPIO_PIN_0;
  myLoRa.DIO0_port = GPIOA;
  myLoRa.DIO0_pin = GPIO_PIN_1;
  myLoRa.hSPIx = &hspi1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  LoRa_reset(&myLoRa);
  HAL_Delay(50);

  HAL_GPIO_WritePin(myLoRa.CS_port, myLoRa.CS_pin, 0);
  HAL_Delay(200);
  printf("--> 0x%2X \r\n",LoRa_read(&myLoRa, RegVersion));
  if( LoRa_init(&myLoRa) == LORA_OK )
	  printf("LoRa Started - Status OK\r\n");

  LoRa_startReceiving(&myLoRa);
  htim3.Instance->ARR=20000;	// Hard coded it to 15 Sec
  	  	  	  	  	  	  	  	// But, Should Implement a Rand Func for selectinga time from 10-25 sec
  	  	  	  	  	  	  	  	// such every node will transmit broadcast msg in rand time
  HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_1);



//  uint8_t msg[]="Hello B !";
//  Packet sp={0};
//  sp.sourId=THIS_NODE;
//  sp.destId='B';
//  sp.control=0;
//  memcpy(sp.data,msg,sizeof(msg));
//  sp.pLen=sizeof(msg);
//  uint8_t tx_buf[150]={0};
//  uint8_t len = serializePacket(&sp, tx_buf, 150);

  //LoRa_transmit(&myLoRa, tx_buf, len, 400);


  while (1)
  {
	  process_rx_ring();

	  if(uart_parse){
		  uart_parse_command(uart_parse_buf, uart_parse_len);
		  uart_parse=0;
	  }
	  // 2. Attempt sending next packet(s) in TX queue up to window size
	  txq_try_send();
	  // 3. Check for retransmissions after ACK timeout
	  txq_check_retransmit();

	  // 4. Periodically prune stale nodes from NodeManager (run every ~10 sec)
	  static uint32_t lastPruneTime = 0;
	  uint32_t now = HAL_GetTick();
	  if ((now - lastPruneTime) > 15000) { // 10 seconds
		  prune_stale_nodes(60000);         // Remove nodes not seen in last 60 seconds
		  lastPruneTime = now;
		  nm_print_nodes();
	  }

	  // 5. Periodically send a broadcast message
	  ///(triggered maybe by htim3 interrupt sets sendBcast flag)
	  if(sendBcast){
		  uint8_t a[2]={0};
		  int b=send_broadcast(a,strlen((char*)a));
		  if(b){
			  printf("Broadcast msg sent at %lu\r\n",HAL_GetTick());
			  sendBcast=0;
		  }


	  }
	  // 6. Optional: Implement a small delay or low power wait here if no real-time requirements
	    HAL_Delay(10);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 64000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 64000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Reset_pin_GPIO_Port, Reset_pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Reset_pin_Pin */
  GPIO_InitStruct.Pin = Reset_pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Reset_pin_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
