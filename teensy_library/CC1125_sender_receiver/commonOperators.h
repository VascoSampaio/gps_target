#define PROCESS_VAL(p) case (p):s = #p;break;

void int2bytearr(byte * arr, int integer, int len) {
  for (int i = 0; i < len; i++)
    arr[i] = 0xFF & integer >> i * 8;
}

void bytearr2int (byte * arr, int& integer, int len) {
  for (int i = 0; i < len; i++)
    integer |= arr[i] << i * 8;
}

enum chip_status
{
  IDLER,
  RX,
  TX,
  FSTXON,
  CALIBRATE,
  SETTLING,
  RXFIFOERROR,
  TXFIFOERROR
};
void printStatus(byte inByte) {
  const char* s = 0;
  switch (inByte) {
      PROCESS_VAL(IDLER);
      PROCESS_VAL(RX);
      PROCESS_VAL(TX);
      PROCESS_VAL(FSTXON);
      PROCESS_VAL(CALIBRATE);
      PROCESS_VAL(SETTLING);
      PROCESS_VAL(RXFIFOERROR);
      PROCESS_VAL(TXFIFOERROR);
  }
  Serial.println(s);
}
