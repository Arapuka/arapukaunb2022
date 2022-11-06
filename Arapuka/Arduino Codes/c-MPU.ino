// MPU - Rotinas para acesso
// CXP - Caixa Preta
// 22/01/2019

// Ler Aceleração, Giro e Mag
// Retorna vetor de 18 bytes
// [axh axl ayh ayl azh azl gxh gxl gyh gyl gzh gzl hxh hxl hyh hyl hzh hzl]
void mpu_rd_ac_gi_mg(byte *vetor){
  byte x,vet[14],st1,st2;
  // Acel e Giro
  mpu_rd_blk(ACCEL_XOUT_H, vet, 14);
  vetor[ 0] = vet[ 0];  //axh
  vetor[ 1] = vet[ 1];  //axl
  vetor[ 2] = vet[ 2];  //ayh
  vetor[ 3] = vet[ 3];  //ayl
  vetor[ 4] = vet[ 4];  //azh
  vetor[ 5] = vet[ 5];  //azl
  // Pular temperatura vet[6] e vet[7]
  vetor[ 6] = vet[ 8];  //gxh
  vetor[ 7] = vet[ 9];  //gxl
  vetor[ 8] = vet[10];  //gyh
  vetor[ 9] = vet[11];  //gyl
  vetor[10] = vet[12];  //gzh
  vetor[11] = vet[13];  //gzl

  // Magnetômetro
  // Data pronto (DRDY=1) ?
  st1=mpu_rd_mg_reg(MAG_ST1);
  if ( (st1&1) == 0)
    st1=mpu_rd_mg_reg(MAG_ST1); //Ler novamente
  mpu_rd_mg_blk(MAG_XOUT_L, vet, 6);

  vetor[12] = vet[1];  //hxh
  vetor[13] = vet[0];  //hxl
  vetor[14] = vet[3];  //hyh
  vetor[15] = vet[2];  //hyl
  vetor[16] = vet[5];  //hzh
  vetor[17] = vet[4];  //hzl
  vetor[17] = (vetor[17] & 0xFE) | (st1&1); //LSbit de hzl = DRDY
  
  // Overflow (HOFL) ==> 9.999
  st2=mpu_rd_mg_reg(MAG_ST2);
  if ((st2&8) == 8){
    vetor[12] = vetor[14] = vetor[16] = 9999>>8;
    vetor[13] = vetor[15] = vetor[17] = 9999&0xFF;
  }
}


////////////////////////////////////////////////////
/////////////// Magnetômetro   /////////////////////
////////////////////////////////////////////////////

// MAG: Realizar Self-Test (ST), prn = imprimir resultados?
// Retorna: TRUE  se passou no teste
//          FALSE se falhou no teste
// vetor[ hx hy hz ] --> espaço para 3 inteiros
byte mpu_mag_self_test(int *vetor, byte prn) {
  byte vet[6],ok;
  int aux[3];
  byte asa[3];
  
  mpu_wr_mg_reg(MAG_CNTL_1, 0x00);  //(1) MODE=0, Magnetometro Power Down
  mpu_wr_mg_reg(MAG_ASTC, 0x64);    //(2) SELF=1
  mpu_wr_mg_reg(MAG_CNTL_1, 0xC);   //(3) BIT=1 (16 bits) e MODE=8 (self test)
  mpu_rd_mg_blk(MAG_XOUT_L, vet, 6);
  aux[0] = (int)((int)(vet [1] << 8) | vet[0]);    //Montar Mag X
  aux[1] = (int)((int)(vet [3] << 8) | vet[2]);    //Montar Mag Y
  aux[2] = (int)((int)(vet [5] << 8) | vet[4]);    //Montar Mag Z
  mpu_wr_mg_reg(MAG_ASTC, 0);       //(2) SELF=0
  mpu_wr_mg_reg(MAG_CNTL_1, 0x00);  //(1) MODE=0, Magnetometro Power Down

  // ASA: Ajuste de sensibilidade
  mpu_mag_rd_rom(asa);    //asa[0]=ASAx, asa[1]=ASAy, asa[2]=ASAz,
  vetor[0]=aux[0]*( ((float)(asa[0]-128)/256.)+1);
  vetor[1]=aux[1]*( ((float)(asa[1]-128)/256.)+1);
  vetor[2]=aux[2]*( ((float)(asa[2]-128)/256.)+1);
  
  ok=TRUE;
  if ( (vetor[0] <=  -200) || (vetor[0] >=  200))  ok=FALSE;  //hx
  if ( (vetor[1] <=  -200) || (vetor[1] >=  200))  ok=FALSE;  //hy
  if ( (vetor[2] <= -3200) || (vetor[2] >= 3200))  ok=FALSE;  //hz

  if (prn==TRUE){ //Imprimir resultados ?
    ser_crlf(1);
    ser_str("\n--- Resultados MAG Self Test ---\n");
    ser_str("hx=");         ser_dec16(aux[0]);
    ser_str("  hy=");       ser_dec16(aux[1]);
    ser_str("  hz=");       ser_dec16(aux[2]);
    ser_str("\nASAx=");     ser_dec8u(asa[0]);
    ser_str("  ASAy=");     ser_dec8u(asa[1]);
    ser_str("  ASAz=");     ser_dec8u(asa[2]);
    ser_str("\nhx=");       ser_dec16(vetor[0]);  //Após ajuste
    ser_str("  hy=");       ser_dec16(vetor[1]);
    ser_str("  hz=");       ser_dec16(vetor[2]);
    ser_str(" ==> ");
    if (ok==TRUE)   ser_str("OK");
    else            ser_str("NOK");     
    ser_str("\n--- Fim Funcao MAG Self Test ---\n\n");
  }
  return ok;
}



// Inicializar Magnetômetro
void mpu_mag_config(void){
  mpu_wr(USER_CTRL, 0x00);          //Desab. modo mestre no mpu
  mpu_wr(INT_PIN_CFG, 0x02);        //Hab.o bypass I2C
  mpu_wr_mg_reg(MAG_CNTL_1, 0x00);  //Magnetometo Power Down
  delay(100);                       //Espera trocar de modo
  mpu_wr_mg_reg(MAG_CNTL_1, 0x16);  //??? Mag. Modo Continuo, 100Hz e 16 bits
  delay(100);                       //espera trocar de modo
}

// Ler Fuse ROM Magnetômetro
// vet = vetor de 3 posições
void mpu_mag_rd_rom(byte *vet){
  mpu_mag_config();                 //Inicializar Mag   
  mpu_wr_mg_reg(MAG_CNTL_1, 0x1F);  //Modo FUSE ROM
  delay(100);                       //Esperar trocar de modo
  mpu_rd_mg_blk(MAG_ASAX, vet, 3);  //Ler ajustes sensiblidade ASAX, ASAY, ASAZ
  mpu_mag_config();                 //Inicializar Mag  
}

//Mag Who am I, deve retornar 0x48
byte mag_whoami(){
  byte who = 0;
  who = mpu_rd_mg_reg(0x00);
  return who;
}

// Ler Magnetômetro
// Retorna vetor de 3 words [hx hy hz]
// Copia DRDY para o último bit de hz
// retorna 0: dado não pronto (DRDY = 0)
// retorna 1: Tudo bem
// retorna 2: overflow (HOFL)
byte mpu_rd_mg_out(int *vetor){
  byte vet[6],st1,st2;
  
  // Data pronto (DRDY=1) ?
  st1=mpu_rd_mg_reg(MAG_ST1);
  if ( (st1&1) == 0)
    st1=mpu_rd_mg_reg(MAG_ST1); //Ler novamente
  
  mpu_rd_mg_blk(MAG_XOUT_L, vet, 6);
  vetor[0] = (int)((int)(vet [1] << 8) | vet[0]);    //Montar Mag X
  vetor[1] = (int)((int)(vet [3] << 8) | vet[2]);    //Montar Mag Y
  vetor[2] = (int)((int)(vet [5] << 8) | vet[4]);    //Montar Mag Z

  // Coloca DRDY no último bit de HZ
  vetor[2] = (vetor[2] & 0xFFFE) | (st1&1); //LSbit de hz = DRDY
  
  // Overflow (HOFL) ==> 9.999
  st2=mpu_rd_mg_reg(MAG_ST2);
  if ((st2&8) == 8){
    vetor[0] = vetor[1] = vetor[2] = 9999;
  }

  if ( (st1&1) == 0)  return 0;   //0=Dado Não pronto
  if ( (st2&8) == 8)  return 2;   //2=Sensor overflow
  return TRUE;                    //1=Tudo certo
}

// (50) Ler registrador do magnetometro
byte mpu_rd_mg_reg(byte reg){
  byte dado;
  twi_start(50);                //START
  twi_er(MAG_I2C_ADDR_WR, 51);  //Endereçar Magnetometro para escrita
  twi_dado_er(reg, 52);         //Informar registrador
  twi_start_rep(53);            //START Repetido
  twi_et(MAG_I2C_ADDR_RD, 54);  //Endereçar Magnetometro para leitura
  dado = twi_dado_et_nack(55);  //Receber dado do magnetometro com NACK
  twi_stop();                   //Gerar STOP para finalizar
  return dado;
}

//(60) lê em bloco os registradores do magnetometro
void mpu_rd_mg_blk(byte reg, byte *dado, byte qtd){
  byte i;
  twi_start(60);                    //START
  twi_er(MAG_I2C_ADDR_WR, 61);      //Endereçar MPU para escrita
  twi_dado_er(reg, 62);             //Informar registrador
  twi_start_rep(63);                //START Repetido
  twi_et(MAG_I2C_ADDR_RD, 64);      //Endereçar Magnetometro para leitura

  for (i=0; i<qtd; i++)
    dado[i] = twi_dado_et_ack(65);  //Receber dados e gerar ACK
    
  dado = twi_dado_et_nack(66);      //Receber último dado e gerar NACK
  twi_stop();                       //Gerar STOP para finalizar
}

// (70) escreve no magnetometro
byte mpu_wr_mg_reg(byte reg, byte dado){
  twi_start(70);                //start
  twi_er(MAG_I2C_ADDR_WR, 71);  //endereça magnetometro para escrita
  twi_dado_er(reg, 72);         //informa o registrador
  twi_dado_er(dado, 73);        //informa o dado
  twi_stop();                   //stop
}


////////////////////////////////////////////////////
/////////// Aceleração e Giro      /////////////////
////////////////////////////////////////////////////

// Ler no MPU a banda do filtro
// Retorna 5, 10, 21, ..., 260 (Escala do Acelerômetro)
int mpu_rd_bw(void){
  byte x;
  x=mpu_rd(CONFIG);
  switch(x){
    case 0:   return 260;
    case 1:   return 184;
    case 2:   return  94;
    case 3:   return  44;
    case 4:   return  21;
    case 5:   return  10;
    case 6:   return   5;
    case 7:   return 999; //Indicar Valor reservado
  }
}

// Ler no MPU a freq de amostragem
// Retorna 100, 200, ..., 1000
// Considera Gyro Rate = 1.000 (DLPF=0)
int mpu_rd_freq(void){
  byte x;
  x=mpu_rd(SMPLRT_DIV);
  x=1000/(1+x);
  return x;
}

// Ler no MPU a escala usada para o acelerômetro
// Retorna 2, 4, 8 ou 16
int mpu_rd_esc_acel(void){
  byte x;
  x=mpu_rd(ACCEL_CONFIG);
  x=(x>>3)&0x7;
  x=2<<x;
  return x;
}

// Ler no MPU a escala usada para o giroscópio
// Retorna 250, 500, 1000 ou 2000 graus/seg
int mpu_rd_esc_giro(void){
  byte x;
  x=mpu_rd(GYRO_CONFIG);
  x=(x>>3)&0x7;
  x=250<<x;
  return x;
}

// Preparar para MPU usar INT4
// Pino PE4 entrada com pullup
// Habilitar INT4 para flanco de descida
// MPU: interrup em baixo com push-pull, pulso de 50 useg
// MPU: Habilitar interrup dado pronto
void mpu_int(void){

  // INT4 = PE4 = Pino 2 --> entrada com pullup 
  DDRE = DDRE & ~(1 << DDE4); //DDD4=0, entrada          
  PORTE = PORTE | (1 << PE4); //Pull-up ligado (PORTE4=1)
  //pinMode(2,INPUT_PULLUP);

  // Preparar interrupção INT4 por flanco de descida
  EICRB = (EICRB | (1<<ISC41)) & ~(1 << ISC40); //INT4 = flanco descida
  EIMSK = EIMSK | (1 << INT4);                  //INT4 habilitada  

  //mpu_wr(INT_PIN_CFG,0x80);  //push-pull, pulso 50 useg
  mpu_wr(INT_PIN_CFG,0x82);  //push-pull, pulso 50 useg
  mpu_wr(INT_ENABLE,1);     //DATA_RDY_EN
}

// Desabilitar MPU que usava INT4
// Desabilitar INT4 para 
// MPU: Desabilitar interrup dado pronto
void mpu_des_int(void){
  EIMSK &= ~(1 << INT4);    //INT4 Desabilitada  
  mpu_wr(INT_ENABLE,0);     //DATA_RDY_EN = 0 (desab)
}

// Ler Temperatura
int mpu_rd_tp(void){
  byte vet[2];
  int x;
  mpu_rd_blk(TEMP_OUT_H, vet, 2);
  x = (int)((vet [0] << 8) | vet[1]);    //Montar Temperatura
  return x;
}

// Ler Aceleração, temperatura e giro
// Retorna vetor de 7 words [ax ay az tp gx gy gz]
void mpu_rd_ac_tp_gi(word *vetor){
  byte i,vet[14];
  mpu_rd_blk(ACCEL_XOUT_H, vet, 14);
  vetor[0] = (int)((vet [0] << 8) | vet[1]);    //Montar Acel X
  vetor[1] = (int)((vet [2] << 8) | vet[3]);    //Montar Acel Y
  vetor[2] = (int)((vet [4] << 8) | vet[5]);    //Montar Acel Z
  vetor[3] = (int)((vet [6] << 8) | vet[7]);    //Montar Temp
  vetor[4] = (int)((vet [8] << 8) | vet[9]);    //Montar Giro x
  vetor[5] = (int)((vet[10] << 8) | vet[11]);   //Montar Giro y
  vetor[6] = (int)((vet[12] << 8) | vet[13]);   //Montar Giro z
}

// Ler Aceleração, giro
// Retorna vetor de 6 words [ax ay az gx gy gz]
void mpu_rd_ac_gi(word *vetor){
  byte i,vet[14];
  mpu_rd_blk(ACCEL_XOUT_H, vet, 14);
  vetor[0] = (int)((vet [0] << 8) | vet[1]);    //Montar Acel X
  vetor[1] = (int)((vet [2] << 8) | vet[3]);    //Montar Acel Y
  vetor[2] = (int)((vet [4] << 8) | vet[5]);    //Montar Acel Z
  vetor[3] = (int)((vet [8] << 8) | vet[9]);    //Montar Giro x
  vetor[4] = (int)((vet[10] << 8) | vet[11]);   //Montar Giro y
  vetor[5] = (int)((vet[12] << 8) | vet[13]);   //Montar Giro z
}

// Acordar o MPU e programar para usar relógio Giro X
void mpu_acorda(void) {
  mpu_wr(PWR_MGMT_1, 1);
}

// Dormir o MPU e programar para usar relógio Giro X
void mpu_dorme(void) {
//  mpu_wr(PWR_MGMT_1, 0x21);  //SLEEP=1 e PLL com Giro X
  mpu_wr(PWR_MGMT_1, 0x41);  //SLEEP=1 e PLL com Giro X
}

// Ler o registrador WHO_AM_I
byte mpu_whoami(void) {
  return mpu_rd(WHO_AM_I);
}

// Colocar o MPU num estado conhecido
// Taxa = 1 kHz, Banda: Acel=5 Hz e Giro=5 Hz e delay=19 mseg
// Taxa de amostragem =  taxa/(1+SMPLRT_DIV) = 1k/10 = 100Hz
//Escalas acel = +/2g e giro = +/-250 gr/s
void mpu_config(void) {

  // Despertar MPU, Relógio = PLL do Giro-x
  mpu_wr(PWR_MGMT_1, 0x01);
  delay(200);       //200ms - Esperar PLL estabilizar

  // 6 ==> Taxa = 1 kHz, Banda: Acel=5 Hz e Giro=5 Hz e delay=19 mseg
  mpu_wr(CONFIG, 6);

  // 9 ==> Taxa de amostragem =  taxa/(1+SMPLRT_DIV) = 1k/10 = 100Hz
  mpu_wr(SMPLRT_DIV, SAMPLE_RT_100Hz);  //Taxa de amostragem = 100 Hz
  //mpu_wr(SMPLRT_DIV, SAMPLE_RT_500Hz);  //Taxa de amostragem = 500 Hz

  // Definir escalas
  mpu_escalas(GIRO_FS_250, ACEL_FS_2G);   //Escalas acel = +/2g e giro = +/-250 gr/s
}

// Selecionar Fundo de Escalas para o MPU
// Acel: 0=+/-2g, 1=+/-4g, 2=+/-8g, 3=+/-16g 
// Gyro: 0=+/-250gr/s, 1=+/-500gr/s, 2=+/-1000gr/s, 3=+/-2000gr/s,  
void mpu_escalas(byte gfs, byte afs) {
  mpu_wr(GYRO_CONFIG, gfs << 3); //FS do Giro
  mpu_wr(ACCEL_CONFIG, afs << 3); //FS do Acel
}

// Selecionar Sample Rate
// Considerando Taxa = 1kHz (Registrador CONFIG)
void mpu_sample_rt(byte sample_rate) {
  mpu_wr(SMPLRT_DIV, sample_rate);  //Taxa de amostragem
}

// MPU: Calibrar
// Faz uma série de leituras e retorna a média
void mpu_calibra(int *vt, word qtd, byte esc_ac, byte esc_gi) {
  long sum[7];  //Acumular para calcular a média
  word aux[7];  //Leituras intermediárias
  word i,j;

  mpu_escalas(esc_gi, esc_ac);          // Menores escalas para maior precisão
  mpu_sample_rt(OP_FREQ);               // Taxa de amostragem de operação
  mpu_int();                            // Habilitar interrupção
  for (i=0; i<7; i++)   sum[i]=0;       //Zerar acumulador
  mpu_dado_ok=FALSE;
  for (i=0; i<qtd; i++){
    //ser_dec16(i);
    //ser_crlf(1);
    while(mpu_dado_ok == FALSE);
    mpu_dado_ok = FALSE;
    mpu_rd_ac_tp_gi(aux);
    for (j=0; j<7; j++) sum[i] += aux[i];
  }

  // Calcular as médias
  for (i=0; i<7; i++) vt[i] = sum[i]/qtd;
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
  mpu_escalas(GIRO_FS_250,ACEL_FS_8G);  //+/- 8g e +/-250gr/seg
  delay(250);                           //Aguardar cofiguração estabilizar
  mpu_rd_ac_gi(&vt[0]);                   //aux1 guarda leitura com self-test desabilitado
  
  // Habilitar Self_Test
  mpu_wr(ACCEL_CONFIG, 0xE0|(ACEL_FS_8G << 3));  //Escala 8g, Self-test Habilitado
  mpu_wr(GYRO_CONFIG, 0xE0|(GIRO_FS_250 << 3));  //Escala 250, Self-test Habilitado
  delay(250);                           //Aguardar cofiguração estabilizar
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
  
  cont=0;
  for (x=18; x<24; x++){
    if (vt[x]>14) cont++;
  }

  if (cont==0)  return TRUE;
  else          return FALSE;
}

///////////////// Rotinas Básicas para MPU

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
  for (i=0; i<qtd; i++)
    dado[i] = twi_dado_et_ack(45);  //Receber dados e gerar ACK
  dado = twi_dado_et_nack(46);  //Receber último dado e gerar NACK
  twi_stop();                   //Gerar STOP para finalizar
}


// MPU: ISR para a interrupção INT4
ISR(INT4_vect){                                
  mpu_dado_ok=TRUE;
  Scp1();
}
