#include "uMB.h"

void word2bytes(uint16_t word, uint8_t* first, uint8_t* second){
  *first = (uint8_t)((word & 0xFF00) >> 8);
  *second = (uint8_t)(word & 0x00FF);
  return;
}

void bytes2word(uint8_t first, uint8_t second, uint16_t* word){
  *word = (uint16_t)(first << 8) + (uint16_t)(second);
  return;
}

/*
 *    ######################
 *    REQUEST IMPLEMENTATION
 *    ######################
 */  
void uMB_req_msg_create(struct uMB_request* req)
{
  req->msg = message_req;
  word2bytes(req->header.tx, &req->msg[0], &req->msg[1]);
  word2bytes(req->header.prot, &req->msg[2], &req->msg[3]);
  req->header.len = 6; // Request always have 12 bytes
  word2bytes(req->header.len, &req->msg[4], &req->msg[5]);
  req->msg[6] = req->unitID;
  req->msg[7] = req->fcode;
  word2bytes(req->address, &req->msg[8], &req->msg[9]);
  word2bytes(req->quantity, &req->msg[10], &req->msg[11]);
  req->msg_len = 12;
  return;
}

void uMB_req_msg_print(struct uMB_request* req){
  if(_UMB_DEBUG!=0x00)
  {
    printf(
      "Request message: \nx%02X x%02X x%02X x%02X x%02X x%02X x%02X x%02X x%02X x%02X x%02X x%02X\n\n", 
      req->msg[0], req->msg[1], req->msg[2], req->msg[3],
      req->msg[4], req->msg[5], req->msg[6], req->msg[7],
      req->msg[8], req->msg[9], req->msg[10], req->msg[11]
    );
  }
  return;
}

/*
 *    #######################
 *    RESPONSE IMPLEMENTATION
 *    #######################
 */
 void uMB_resp_msg_create(struct uMB_response* resp)
{
  resp->msg = message_resp;
  word2bytes(resp->header.tx, &resp->msg[0], &resp->msg[1]);
  word2bytes(resp->header.prot, &resp->msg[2], &resp->msg[3]);
  word2bytes(resp->header.len, &resp->msg[4], &resp->msg[5]);
  resp->msg[6] = resp->unitID;
  resp->msg[7] = resp->fcode;
  return;
}

uint8_t uMB_check_id(uint8_t* msg_in, struct uMB_response* resp){
  uint8_t req_id = msg_in[6];
  // Response Id should be defined in uMB_response declaration
  if(req_id == resp->unitID)
      return 1;
  else
      return 0;
}

 void uMB_resp_parse(uint8_t* msg_in, uint16_t data[], struct uMB_response* resp)
{ 
  uint16_t quantity = 0;
  // TX : byte 0, 1:
  bytes2word(msg_in[0], msg_in[1], &resp->header.tx);
  word2bytes(resp->header.tx, &resp->msg[0], &resp->msg[1]);
  // PROT: byte 2, 3:
  bytes2word(msg_in[2], msg_in[3], &resp->header.prot);
  resp->msg[2] = msg_in[2]; resp->msg[3] = msg_in[3];
  // LEN : byte 4, 5 (calculated in the end for response):
  resp->header.len = 0x0000;
  // Unit ID : byte 6:
  resp->unitID = msg_in[6];
  resp->msg[6] = resp->unitID; 
  // Function nr : byte 7:
  resp->fcode = msg_in[7];
  resp->msg[7] = resp->fcode;
  // Address : only request:
  bytes2word(msg_in[8], msg_in[9], &resp->address);
  // Quantity : byte 8:
  bytes2word(msg_in[10], msg_in[11], &quantity);
  resp->quantity = (uint8_t)(quantity*2);
  resp->msg[8] = resp->quantity;
  // Register Values : All next bytes:
  resp->msg_len = 9;
  for(uint8_t i = 0; i<quantity; i++){
    word2bytes(data[i+resp->address], &resp->msg[9+(i*2)], &resp->msg[10+(i*2)]);
    resp->header.len = (uint16_t) 10+i;
    resp->msg_len += 2;
  }
  word2bytes(resp->header.len, &resp->msg[4], &resp->msg[5]);
  return;
}

void uMB_resp_msg_print(struct uMB_response* resp){
  if(_UMB_DEBUG!=0x00)
  {
  printf("Response message: \n");
  for(uint8_t i = 0; i<9+resp->quantity; i++)
    printf("x%02X ", resp->msg[i]);
  printf("\nWith length: %d\n", resp->msg_len);
  }
  return;
}
