// KXC - MPU - TWI
// 27/09/2020
// Fuções para TWI e MPU

/////////////// MPU 6050 - Constantes
#define MPU_ADR  0x69  //Endereço MPU-6050
#define MPU_EWR  0xD2  //MPU para escrita (0x69<<1)
#define MPU_ERD  0xD3  //MPU para leitura (0x69<<1 + 1)

//Escalas para Giroscópio
#define GIRO_FS_250  0   // +/- 250 graus/seg
#define GIRO_FS_500  1   // +/- 500 graus/seg
#define GIRO_FS_1000 2   // +/- 1000 graus/seg
#define GIRO_FS_2000 3   // +/- 2000 graus/seg

//Escalas para Acelerômetro
#define ACEL_FS_2G  0   // +/- 2g
#define ACEL_FS_4G  1   // +/- 4g
#define ACEL_FS_8G  2   // +/- 8g
#define ACEL_FS_16G 3   // +/- 16g

// Registradores do MPU-6050 que foram usados
#define SELF_TEST_X      0x0D
#define SELF_TEST_Y      0x0E
#define SELF_TEST_Z      0x0F
#define SELF_TEST_A      0x10
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define FIFO_EN          0x23
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B   //ax
#define TEMP_OUT_H       0x41   //Temp
#define GYRO_XOUT_H      0x43   //gx
#define USER_CTRL        0x6A
#define PWR_MGMT_1       0x6B
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I         0x75

/////////////// TWI - Códigos de Status
#define TWI_START_OK      8     //Start OK
#define TWI_START_REP_OK  0x10  //Start Repetido OK
#define TWI_SLA_WR_ACK    0x18  //EET enviado e ACK recebido
#define TWI_SLA_WR_NACK   0x20  //EET enviado e NACK recebido
#define TWI_TX_DATA_ACK   0x28  //Dado enviado e ACK recebido
#define TWI_SLA_RD_ACK    0x40  //EER enviado e ACK recebido
#define TWI_SLA_RD_NACK   0x48  //EER enviar e NACK recebido 
#define TWI_RX_DATA_NACK  0x58  //Dado recebido e NACK gerado
#define TWI_RX_DATA_ACK   0x50  //Dado recebido e ACK gerado
#define TWI_TMI_OUT       10000 //Time out

#define TWI_ERRO_1  1   //Erro ao gerar START
#define TWI_ERRO_2  2   //Erro ao gerar START Repetido
#define TWI_ERRO_3  3   //Erro Escravo Receptor endereçado (ER) não enviou ACK
#define TWI_ERRO_4  4   //Erro Escravo Transmissor endereçado (ET) não enviou ACK
#define TWI_ERRO_5  5   //Erro Escravo Receptor (ER) não enviou ACK após envio do dado
#define TWI_ERRO_6  6   //Erro ao receber um dado do Escravo Transmissor (ET) e gerar um ACK
#define TWI_ERRO_7  7   //Erro ao receber um dado do Escravo Transmissor (ET) e gerar um NACK
#define TWI_ERRO_8  8   //Erro ao esperar TWINT - Timeout esperando TWINT ir para 1


///////////////////////////////////////////////////////////////////////////
/////////////////////////// MPU ///////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

// Indicar a posição do KXC
byte mpu_pos(int ay){
  word ang;
  ang=mpu_ang_y(ay);
  if     (ang < HORIZ_MIN) return  VERT_OK;
  else if(ang > HORIZ_MAX) return  VERT_INV;
  else                     return  HORIZ;
}


// Calcular inclinação usando só o eixo Y
// Em décimos de graus
word mpu_ang_y(int ay){
  word ang;
  int vy;   //ay, posição vertical
  eeprom_rd_vet_int(EE_AY_VERT, &vy, 1); //Ler posição vertical na EEPROM
  ang = 10*RAD_2_GRAD*acos((float)ay/(float)vy);
  //ser_str("\n ay=");   ser_dec16(ay);
  //ser_str(" vy=");   ser_dec16(vy);
  //ser_str(" ang=");  ser_dec16(ang);
  //ser_crlf(1);
  return ang;
}


// Esperar até KXC ficar parado (estável)
// Pára se acontece "vz" medidas sequenciais de agt menor que MPU_ESTAVEL
byte mpu_estavel(byte vz){
  byte cont=0;
  while(cont<vz){
    if (mpu_agito()<MPU_ESTAVEL){
      cont++;
      ser_char('.');
    }
    else{
      if (cont>0) cont--;
      ser_char('*');
    }
  }
  return TRUE;
}

// Agito
// Verificar se o KXC está parado
byte mpu_agito(void){
  byte i, j;
  word agt;
  int vt[4];
  int vet_max[3];
  int vet_min[3];
  int vet_dt[3];

  // Ler deltas da EEPROM
  for (i=0; i<3; i++){
    vet_dt[i]=eeprom_rd_word(EE_DT_AX+2*i);
  }

  // Verificar se fez cálculo de offset (COF)
  if (vet_dt[0]==-1 && vet_dt[1]==-1){
    ser_str("?COF?\n");
    return 255;
  }

  // Inicializar
  vet_max[0]=vet_max[1]=vet_max[2]=-32767;
  vet_min[0]=vet_min[1]=vet_min[2]=+32767;
  
  // Fazer 16 medidas
  espera_100ms(1); //Sincronizar com flags de 100ms
  for (i=0; i<16; i++){
    espera_100ms(1);
    mpu_media(vt,1);
    for (j=0; j<3; j++){
      if (vet_max[j]<vt[j])  vet_max[j]=vt[j];
      if (vet_min[j]>vt[j])  vet_min[j]=vt[j];
    }
  }
  
  // Calcular as diferenças
  for (i=0; i<3; i++){
    vt[i]=vet_max[i]-vet_min[i];
  }

  // Calcular relação dif/delta
  for (i=0; i<3; i++){
    if (vet_dt[i] != 0)   vt[i]=vt[i]/vet_dt[i];
  }
  agt=vt[0];
  for (i=0; i<3; i++){
    if (agt<vt[i]) agt=vt[i];
  }
  if (agt>200)  agt=200;
  return agt;
}

// Ler no MPU a escala usada para o acelerômetro
// Retorna 0,1,2,3 ==> 2, 4, 8 ou 16 g
int mpu_rd_esc_acel(void){
  byte x;
  x=mpu_rd(ACCEL_CONFIG);
  x=(x>>3)&3;
  return x;
}

// Ler no MPU a escala usada para o giroscópio
// Retorna Retorna 0,1,2,3 ==> 250, 500, 1000 ou 2000 graus/seg
int mpu_rd_esc_giro(void){
  byte x;
  x=mpu_rd(GYRO_CONFIG);
  x=(x>>3)&3;
  return x;
}

// Ler aceleração, temperatura e calcular a média
// vetor deve ter o tamanho 4 [ax ay az tp]
void mpu_media(int *vetor, int qtd){
  long ax=0,ay=0,az=0,tp=0;
  byte vet[8];
  int i;
  for (i=0; i<qtd; i++){
    mpu_rd_blk(ACCEL_XOUT_H, vet, 8);
    ax += (int)((vet[ 0] << 8) | vet[ 1]);    //Montar Acel X
    ay += (int)((vet[ 2] << 8) | vet[ 3]);    //Montar Acel Y
    az += (int)((vet[ 4] << 8) | vet[ 5]);    //Montar Acel Z
    tp += (int)((vet[ 6] << 8) | vet[ 7]);    //Montar Acel Z
  }
  vetor[0]=ax/qtd;
  vetor[1]=ay/qtd;
  vetor[2]=az/qtd;
  vetor[3]=tp/qtd;
}

// SFT: Ler acel-giro (Não lê temperatura)
// Usada na função Self Test
// vt precisa ter tamanho 6
void mpu_rd_ac_gi(int *vt){
  byte vet[14];
  mpu_rd_blk(ACCEL_XOUT_H, vet, 14);
  vt[0] = (int)((vet[ 0] << 8) | vet[ 1]);    //Montar Acel X
  vt[1] = (int)((vet[ 2] << 8) | vet[ 3]);    //Montar Acel Y
  vt[2] = (int)((vet[ 4] << 8) | vet[ 5]);    //Montar Acel Z
  //vt3] = (int)((vet[ 6] << 8) | vet[ 7]);    //Pulou - Montar Temp
  vt[3] = (int)((vet[ 8] << 8) | vet[ 9]);    //Montar Giro X
  vt[4] = (int)((vet[10] << 8) | vet[11]);    //Montar Giro Y
  vt[5] = (int)((vet[12] << 8) | vet[13]);    //Montar Giro Z
}



// Ler aceleração e calcular a média
void mpu_acel_media(int *vet, int qtd){
  long ax=0,ay=0,az=0;
  int i;
  for (i=0; i<qtd; i++){
       mpu_rd_acel(vet);
       ax += vet[0];
       ay += vet[1];
       az += vet[2];
  }
  vet[0]=ax/qtd;
  vet[1]=ay/qtd;
  vet[2]=az/qtd;
}

// Ler aceleração
// vt precisa ter tamanho 3
void mpu_rd_acel(int *vt){
  byte vet[6];
  mpu_rd_blk(ACCEL_XOUT_H, vet, 6);
  vt[0] = (int)((vet[ 0] << 8) | vet[ 1]);    //Montar Acel X
  vt[1] = (int)((vet[ 2] << 8) | vet[ 3]);    //Montar Acel Y
  vt[2] = (int)((vet[ 4] << 8) | vet[ 5]);    //Montar Acel Z
}

// Ler giroscópio e calcular a média
void mpu_giro_media(int *vet, int qtd){
  long gx=0,gy=0,gz=0;
  int i;
  for (i=0; i<qtd; i++){
       mpu_rd_giro(vet);
       gx += vet[0];
       gy += vet[1];
       gz += vet[2];
  }
  vet[0]=gx/qtd;
  vet[1]=gy/qtd;
  vet[2]=gz/qtd;
}

// Ler giroscópio
// vt precisa ter tamanho 3
void mpu_rd_giro(int *vt){
  byte vet[6];
  mpu_rd_blk(GYRO_XOUT_H, vet, 6);
  vt[0] = (int)((vet[ 0] << 8) | vet[ 1]);    //Montar Giro X
  vt[1] = (int)((vet[ 2] << 8) | vet[ 3]);    //Montar Giro Y
  vt[2] = (int)((vet[ 4] << 8) | vet[ 5]);    //Montar Giro Z
}

// Ler temperatura
void mpu_rd_temp(int *vt){
  byte vet[2];
  mpu_rd_blk(TEMP_OUT_H, vet, 2);
  vt[0] = (int)((vet[0] << 8) | vet[1]);    //Montar Giro Temp
}

// Calcular media de leituras ax ay az gx gy gz
void mpu_ac_gi_media(int *vetor, long qtd){
  long i,ax=0,ay=0,az=0,gx=0,gy=0,gz=0;
  int vet[7];
  for (i=0; i<qtd; i++){
    mpu_rd_ac_tp_gi(vet);
    ax += vet[0];  ay += vet[1];  az += vet[2];  
    gx += vet[4];  gy += vet[5];  gz += vet[6];  
  }
  vetor[0]=ax/qtd;    vetor[1]=ay/qtd;      vetor[2]=az/qtd;
  vetor[3]=gx/qtd;    vetor[4]=gy/qtd;      vetor[5]=gz/qtd;
}


// Ler acel-temp-giro
// vt precisa ter tamanho 7
void mpu_rd_ac_tp_gi(int *vt){
  byte vet[14];
  mpu_rd_blk(ACCEL_XOUT_H, vet, 14);
  vt[0] = (int)((vet[ 0] << 8) | vet[ 1]);    //Montar Acel X
  vt[1] = (int)((vet[ 2] << 8) | vet[ 3]);    //Montar Acel Y
  vt[2] = (int)((vet[ 4] << 8) | vet[ 5]);    //Montar Acel Z
  vt[3] = (int)((vet[ 6] << 8) | vet[ 7]);    //Montar Temp
  vt[4] = (int)((vet[ 8] << 8) | vet[ 9]);    //Montar Giro X
  vt[5] = (int)((vet[10] << 8) | vet[11]);    //Montar Giro Y
  vt[6] = (int)((vet[12] << 8) | vet[13]);    //Montar Giro Z
}

// Configurar MPU
// Colocar o MPU num estado conhecido
void mpu_config(void) {
  mpu_acorda();           // Despertar MPU, Relógio = Giro x
  espera_100ms(2);        //200ms - Esperar PLL estabilizar
  mpu_wr(CONFIG,6);       //Fs=1 kHz, BW = 5 Hz (acel e giro)
  mpu_wr(SMPLRT_DIV,99);  //Taxa=10 Hz =Fs/(1+99);
  mpu_esc_acel(2);        //Acel em +/- 2g
  mpu_esc_giro(250);      //Giro em +/- 250graus/seg
}


// Programar escala do acelerômetro
// Valores 2, 4, 6, 8 g
// Programa em zero os bits de Self Test
void mpu_esc_acel(byte acel){
  byte x;
  switch(acel){
    case  2: x=0;  break;
    case  4: x=1;  break;
    case  8: x=2;  break;
    case 16: x=3;  break;
    default: x=0;  break; //<<==Default
  }
  mpu_wr(ACCEL_CONFIG, x<<3);
}

// Programar escala do giroscópio
// Valores 250, 500, 1000, 2000 graus/seg
// Programa em zero os bits de Self Test
void mpu_esc_giro(byte giro){
  byte x;
  switch(giro){
    case  250: x=0;  break;
    case  500: x=1;  break;
    case 1000: x=2;  break;
    case 2000: x=3;  break;
    default: x=0;  break; //<<==Default
  }
  mpu_wr(GYRO_CONFIG, x<<3);
}


// Ler o registrador WHO_AM_I
byte mpu_who_am_i(void) {
  return mpu_rd(WHO_AM_I);
}

// Acordar o MPU e programar para usar relógio Giro X
void mpu_acorda(void) {
  mpu_wr(PWR_MGMT_1, 1);
}

// Dormir o MPU e programar para usar relógio Giro X
void mpu_dorme(void) {
  mpu_wr(PWR_MGMT_1, 0x21);  //SLEEP=1 e relógio Giro X
}

// MPU: Realizar Self-Test (ST), prn = imprimir resultados?
// Retorna: TRUE  se passou no teste
//          FALSE se falhou no teste
//
//     - Self test off -  - Self test on -    -Reg. Self test -    -Calculo tolerância 
// vt[ ax ay az gx gy gz  ax ay az gx gy gz   ax ay az gx gy gz    ax ay az gx gy gz]
//     0                  6                   12                   18
// vt deve ter espaço para 24 inteiros
byte mpu_self_test(int *vt, byte prn) {
  byte x,cont;
  byte aux[6];   //Leitura dos registradores de Self test
  float  gxf, gyf, gzf, axf, ayf, azf; //Factory Trim
  
  //Acertar escalas e desligar Self Test
  mpu_esc_acel(ACEL_FS_8G);   //Self test pede 8G
  mpu_esc_giro(GIRO_FS_250);  //Self test pede 250gr/s
  espera_100ms(1);          //Aguardar cofiguração estabilizar
  mpu_rd_ac_gi(&vt[0]);     //aux1 guarda leitura com self-test desabilitado
  
  // Habilitar Self_Test
  mpu_wr(ACCEL_CONFIG, 0xE0|(ACEL_FS_8G << 3));  //Escala 8g, Self-test Habilitado
  mpu_wr(GYRO_CONFIG, 0xE0|(GIRO_FS_250 << 3));  //Escala 250, Self-test Habilitado
  espera_100ms(1);          //Aguardar cofiguração estabilizar
  mpu_rd_ac_gi(&vt[6]);                   //aux2 guarda leitura com self-test desabilitado

  // Leitura dos resultados do self-test - Montar valores
  mpu_rd_blk(SELF_TEST_X, aux, 4);
  vt[12] = (0x1C&(aux[0]>>3)) | (0x3&(aux[3]>>4));  //XA_TEST
  vt[13] = (0x1C&(aux[1]>>3)) | (0x3&(aux[3]>>2));  //YA_TEST
  vt[14] = (0x1C&(aux[2]>>3)) | (0x3&(aux[3]>>0));  //ZA_TEST
  vt[15] = aux[0]&0x1F;                             //XG_TEST
  vt[16] = aux[1]&0x1F;                             //YG_TEST
  vt[17] = aux[2]&0x1F;                             //ZG_TEST

  // Calcular os Factory Trim
  axf = (4096.0*0.34) * (pow((0.92/0.34) , (((float)vt[12] - 1.0) / 30.0)));
  ayf = (4096.0*0.34) * (pow((0.92/0.34) , (((float)vt[13] - 1.0) / 30.0)));
  azf = (4096.0*0.34) * (pow((0.92/0.34) , (((float)vt[14] - 1.0) / 30.0)));
  gxf = ( 25.0 * 131.0) * (pow( 1.046 , ((float)vt[15] - 1.0) ));
  gyf = (-25.0 * 131.0) * (pow( 1.046 , ((float)vt[16] - 1.0) ));
  gzf = ( 25.0 * 131.0) * (pow( 1.046 , ((float)vt[17] - 1.0) ));

  // Se registrador = 0 --> Factory Trim = 0
  if (vt[12] == 0) axf = 0;
  if (vt[13] == 0) ayf = 0;
  if (vt[14] == 0) azf = 0;
  if (vt[15] == 0) gxf = 0;
  if (vt[16] == 0) gyf = 0;
  if (vt[17] == 0) gzf = 0;

  // Calcular as Percentagens de Alteração
  vt[18] = 100.0 * ((float)(vt[ 6] - vt[0]) - axf ) / axf;
  vt[19] = 100.0 * ((float)(vt[ 7] - vt[1]) - ayf ) / ayf;
  vt[20] = 100.0 * ((float)(vt[ 8] - vt[2]) - azf ) / azf;
  vt[21] = 100.0 * ((float)(vt[ 9] - vt[3]) - gxf ) / gxf;
  vt[22] = 100.0 * ((float)(vt[10] - vt[4]) - gyf ) / gyf;
  vt[23] = 100.0 * ((float)(vt[11] - vt[5]) - gzf ) / gzf;

  /*
  if (prn==TRUE){ //Imprimir resultados ?
    ser_crlf(1);
    ser_str("\n--- Resultados Funcao Self Test ---\n");
    ser_spc(17);
    ser_str("ax     ay     az     gx     gy     gz\n");
    // Self test off
    ser_str("Self Test off: ");
    for (x=0; x<6; x++){  ser_dec16(vt[x]);   ser_spc(1); }
    ser_crlf(1);
    // Self test on 
    ser_str("Self Test on:  ");
    for (x=6; x<12; x++){  ser_dec16(vt[x]);   ser_spc(1); }
    ser_crlf(1);
    // Reg de Self test
    ser_str("Reg Self Test: ");
    for (x=12; x<18; x++){  ser_dec16(vt[x]);   ser_spc(1); }
    ser_crlf(1);
    //Factory trim
    ser_str("Factory Trim:  ");
    ser_float(axf,4); ser_spc(1);  
    ser_float(ayf,4); ser_spc(1);  
    ser_float(azf,4); ser_spc(1);  
    ser_float(gxf,4); ser_spc(1);  
    ser_float(gyf,4); ser_spc(1);  
    ser_float(gzf,4); ser_spc(1);  
    ser_crlf(1);
    // Resultado Tolerância
    ser_str("Resultados:    ");
    for (x=18; x<24; x++){  ser_dec16(vt[x]);   ser_spc(1); }
    ser_str("\n--- Fim Funcao Self Test ---\n");
    ser_crlf(1);
  }
  */

  cont=0;
  for (x=18; x<24; x++){
    if (vt[x]>14) cont++;
  }

  if (cont==0)  return TRUE;
  else          return FALSE;
}




/////////////////////////////////////////////////////////////
///////////////// Rotinas Básicas para MPU //////////////////
/////////////////////////////////////////////////////////////

// (10) Escrever num registrador do MPU
void mpu_wr(byte reg, byte dado) {
  twi_start(10);          //START
  twi_er(MPU_EWR, 11);    //Endereçar MPU para escrita
  twi_dado_er(reg, 12);   //Informar acesso ao PWR_MGMT_1 (0x6B)
  twi_dado_er(dado, 13);  //Selecionar PLL eixo X como referência
  twi_stop();             //Gerar STOP para finalizar
}

// (20) Ler um registrador do MPU
byte mpu_rd(byte reg) {
  uint8_t dado;
  twi_start(20);                //START
  twi_er(MPU_EWR, 21);           //Endereçar MPU para escrita
  twi_dado_er(reg, 22);         //Informar registrador
  twi_start_rep(23);            //START Repetido
  twi_et(MPU_ERD, 24);           //Endereçar MPU para leitura
  dado = twi_dado_et_nack(25);  //Receber dado do MPU com NACK
  twi_stop();                    //Gerar STOP para finalizar
  return dado;
}

// (30) Escrever um bloco de dados no MPU a partir de um registrador
void mpu_wr_blk(byte reg, byte *dado, byte qtd) {
  uint8_t i;
  twi_start(30);                //START
  twi_er(MPU_EWR, 31);          //Endereçar MPU para escrita
  twi_dado_er(reg, 32);         //Informar acesso ao PWR_MGMT_1 (0x6B)
  for (i = 0; i < qtd; i++)
    twi_dado_er(dado[i], 33);   //Selecionar PLL eixo X como referência
  twi_stop();                   //Gerar STOP para finalizar
}

// (40) Ler um bloco do MPU a partir de um registrador
void mpu_rd_blk(byte reg, byte *dado, byte qtd) {
  byte i;
  twi_start(40);                //START
  twi_er(MPU_EWR, 41);          //Endereçar MPU para escrita
  twi_dado_er(reg, 42);         //Informar registrador
  twi_start_rep(43);            //START Repetido
  twi_et(MPU_ERD, 44);          //Endereçar MPU para leitura
  for (i=0; i<qtd-1; i++)
    dado[i] = twi_dado_et_ack(45);  //Receber dados e gerar ACK
  dado[i] = twi_dado_et_nack(46);  //Receber último dado e gerar NACK
  twi_stop();                   //Gerar STOP para finalizar
}

///////////////////////////////////////////////////////////////////////////
/////////////////////////// TWI ///////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

// Programar frequencia do SCL para 100kHz
// Com relógio = 4 MHz
void twi_100k_4MHz(void){
  TWBR = 3;  //SCL = 100 kHz com relógio de 4 MHz
  TWSR = 0;  //TWPS1 = TWPS0 = 0 Pré-escalonador
}

// Programar frequencia do SCL para 125kHz
// Com relógio = 4 MHz
void twi_125k_4MHz(void){
  TWBR = 2;  //SCL = 125 kHz com relógio de 4 MHz
  TWSR = 0;  //TWPS1 = TWPS0 = 0 Pré-escalonador
}

// Programar frequencia do SCL para 166kHz
// Com relógio = 4 MHz
void twi_166k_4MHz(void){
  TWBR = 1;  //SCL = 166 kHz com relógio de 4 MHz
  TWSR = 0;  //TWPS1 = TWPS0 = 0 Pré-escalonador
}

// Programar frequencia do SCL para 250kHz
// Com relógio = 4 MHz
void twi_250k_4MHz(void){
  TWBR = 0;  //SCL = 250 kHz com relógio de 4 MHz
  TWSR = 0;  //TWPS1 = TWPS0 = 0 Pré-escalonador
}


// Programar frequencia do SCL para 25kHz
// Com relógio = 1 MHz
void twi_25k_1MHz(void){
  TWBR = 12; //SCL = 25 kHz em 1 com relógio de 1 MHz
  TWSR = 0;  //TWPS1 = TWPS0 = 0 Pré-escalonador
}

// Programar frequencia do SCL 6,25kHz
// Com relógio = 1 MHz
void twi_6k_1MHz(void){
  TWBR = 72; //SCL = 6,25 kHz com relógio de 1 MHz
  TWSR = 0;  //TWPS1 = TWPS0 = 0 Pré-escalonador  
}

// Programar frequência de SCL = 55,55 kHz (T = 18 useg)
// Só usar com relógio = 1 MHz 
void twi_55k_1MHz(void){
  TWBR = 1;  //SCL = 55,55 kHz
  TWSR = 0;  //TWPS1 = TWPS0 = 0 Pré-escalonador  
}

//Gerar um START no TWI
void twi_start (byte ix){
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);  //Enviar START
  while ( !(TWCR & (1<<TWINT))) ;              //Esperar TWINT = 1
  if ( (TWSR & 0xF8) != TWI_START_OK)  twi_erro(TWI_ERRO_1, ix);
}

//Gerar um START Repetido no TWI
void twi_start_rep (byte ix){
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);  //Enviar START
  espera_twint(ix+1);
  if ( (TWSR & 0xF8) != TWI_START_REP_OK)  twi_erro(TWI_ERRO_2, ix);
}

//Enviar STOP
void twi_stop (void){
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);  // Enviar STOP
  delay (1); //(para CLK=1 MHz) atraso para que STOP seja percebido
}

// Enviar endereço de Escrita do Escravo (ER) e esperar ACK
// Retorna TRUE se escravo gerou ACK
// Retorna FALSE se gerou NACK (não gera msg de erro)
// ee = endereço do escravo
byte twi_er_check (byte ee, byte ix){
  TWDR = ee;                      //Endereço do  escravo
  TWCR = (1<<TWINT) | (1<<TWEN);  //Enviar endereço
  while ( !(TWCR & (1<<TWINT)));  //Esperar TWINT = 1
  if ( (TWSR & 0xF8) != TWI_SLA_WR_ACK)  return FALSE;
  return TRUE;
}


// Enviar endereço de Escrita do Escravo (ER) e esperar ACK
// eer = endereço do escravo receptor
void twi_er (byte eer, byte ix){
  TWDR = eer;                     //Endereço de escrita no escravo
  TWCR = (1<<TWINT) | (1<<TWEN);  //Enviar endereço
  while ( !(TWCR & (1<<TWINT)));  //Esperar TWINT = 1
  if ( (TWSR & 0xF8) != TWI_SLA_WR_ACK)  twi_erro(TWI_ERRO_3, ix);
}

// Enviar endereço de Leitura do Escravo (ET) e esperar ACK
// eet = endereço do escravo transmissor
void twi_et (byte eet, byte ix){
  TWDR = eet;                     //Endereço de leitura do escravo
  TWCR = (1<<TWINT) | (1<<TWEN);  //Enviar endereço
  while ( !(TWCR & (1<<TWINT)));  //Esperar TWINT = 1
  if ( (TWSR & 0xF8) != TWI_SLA_RD_ACK)  twi_erro(TWI_ERRO_4,ix);
}

//Enviar dado para escravo previamente endereçado
void twi_dado_er (byte dado, byte ix){
  TWDR = dado;                      //dado a ser enviado
  TWCR = (1<<TWINT) | (1<<TWEN);    //Enviar dado
  espera_twint(ix+1);
  if ( (TWSR & 0xF8) != TWI_TX_DATA_ACK) twi_erro(TWI_ERRO_5,ix);     // Recebeu ACK ?
}

// Receber dado e gerar ACK
byte twi_dado_et_ack(byte ix){
  TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN);  // Iniciar recepção (TWEA=1 --> ACK)
  espera_twint(ix+1);
  if ( (TWSR & 0xF8) != TWI_RX_DATA_ACK) twi_erro(TWI_ERRO_6,ix);
  return TWDR;
}

// Receber dado e gerar NACK
byte twi_dado_et_nack(byte ix){
  TWCR = (1<<TWINT) | (1<<TWEN);      // Iniciar recepção (TWEA=0 --> NACK)
  //while ( !(TWCR & (1<<TWINT))) ;   // Esperar TWINT = 1
  espera_twint(ix+1);
  if ( (TWSR & 0xF8) != TWI_RX_DATA_NACK) twi_erro(TWI_ERRO_7,ix);
  return TWDR;
}

// Esperar TWINT
byte espera_twint(byte ix){
  word tout=0;  
  char msg[50];
  while ( !(TWCR & (1<<TWINT))){    //Esperar TWINT = 1
    if (tout++==1000){
      twi_erro(TWI_ERRO_8,ix);
      //sprintf(msg,"(index=%d) Nunca chegou TWINT!",ix);
      //Serial.println(msg);
      return(FALSE);
    }
  }
  return(TRUE);
}

// ERROS no TWI (I2C)
// 1 = Erro ao gerar START
// 2 = Erro ao gerar START Repetido
// 3 = Erro Escravo Receptor endereçado (ER) não enviou ACK
// 4 = Erro Escravo Transmissor endereçado (ET) não enviou ACK
// 5 = Erro Escravo Receptor (ER) não enviou ACK após envio do dado
// 6 = Erro ao receber um dado do Escravo Transmissor (ET) e gerar um ACK
// 7 = Erro ao receber um dado do Escravo Transmissor (ET) e gerar um NACK
// 8 = Erro ao esperar TWINT - Timeout esperando TWINT ir para 1
void twi_erro(int cod, int ix){
  char msg[50];
  ser_str("Erro TWI Nr=");  ser_dec16unz(cod);
  ser_str(" Ixr=");         ser_dec16unz(ix);
  ser_str(" TWSR=");        ser_hex8(TWSR&0xF8);
  ser_str("Parou!");
  while (1) ;   //prender execução
}
