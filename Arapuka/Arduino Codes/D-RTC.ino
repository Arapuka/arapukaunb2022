// KXC - RTC
// 04/12/2020
// Fuções para o RTC DS3231

/////////////// RTC DS 3231 - Constantes
#define RTC_ADR  0x68  //Endereço RTC DS3231
#define RTC_EWR  0xD0  //RTC para escrita (0x68<<1)
#define RTC_ERD  0xD1  //RTC para leitura (0x68<<1 + 1)

///////////////////////////////////////////////////////////////////////////
/////////////////////////// RTC ///////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

// Temperatura em graus (despreza fração)
// Consulta BUSY e inicia conversão
// TRUE se conseguiu
// FALSE não conseguiu, BUSY=1 (time out)
byte rtc_temp(byte *tp){
  byte to=0,x;

  // Esperar BUSY=0, se timeout retorna FALSE
  while(TRUE){
    if ( (rtc_rd(0xF)&BIT2) == 0) break;  //BUSY=0
    if ( ++to == 10)  return FALSE;       //Timeout    
  }
  // Disparar conversão de temperatura
  x=rtc_rd(0xE);
  rtc_wr(0xE, x|BIT5);
  while ( (rtc_rd(0xE)&BIT5) == BIT5) ;  // Esperar CONV=0
  //*tp=(rtc_rd(0x11) & ~BIT7);
  *tp=rtc_rd(0x11);
  return TRUE; 
}

// Faz todos os preparativos para ligar o KXC a cada hora
// minu = minuto (decimal) em que deve ser ligado
byte rtc_kxc_on_hora(byte minu){
  rtc_alm_flag(1);                  //Apagar flag alarme 1
  if (rtc_alm_hora(1,minu)==FALSE) return FALSE;
  rtc_ALM(1);                       //Habilitar saída do alarme
  return TRUE;
}


// Faz todos os preparativos para ligar o KXC a cada minuto
// seg = segundo (decimal) em que deve ser ligado
byte rtc_kxc_on_minuto(byte seg){
  rtc_alm_flag(1);              //Apagar flag alarme 1
  if (rtc_alm_min(seg)==FALSE) return FALSE;
  rtc_ALM(1);                 //Habilitar saída do alarme
  return TRUE;
}

// Programar alarme 1 ou 2
// Programa os minutos, ou seja, acontece a cada hora
// Já ativa o alarme (A1M2=1 ou A2M2=1)
// Espera-se que minu esteja na base 10
byte rtc_alm_hora(byte qual, byte minu){
  char x,minu_bcd;
  if (minu>59)  return FALSE;
  minu_bcd=(minu/10)<<4;
  minu_bcd=minu%10;
  if (qual==1)  rtc_wr(0x08, minu_bcd);
  if (qual==2)  rtc_wr(0x0B, minu_bcd);
  return TRUE;
}

// Programar alarme 1 para acontecer a cada minuto
// O alarme 2 não tem esse recurso
// Programa os segundos, ou seja, acontece a cada minuto
// Espera-se que seg esteja em decinal
byte rtc_alm_min(byte seg){
  char x,seg_bcd;
  //ser_str("recebeu=");   ser_dec8(seg);  ser_crlf(1);
  if (seg>59)  return FALSE;
  seg_bcd=(seg/10)<<4;
  seg_bcd+=seg%10;
  rtc_wr(0x07, seg_bcd);  //A1M1=0
  rtc_wr(0x08, 0x80);     //A1M2=1
  rtc_wr(0x09, 0x80);     //A1M3=1
  rtc_wr(0x0A, 0x80);     //A1M4=1
  //ser_str("alm=");   ser_hex8(seg_bcd);
  //ser_crlf(1);
  return TRUE;
}

// Mascarar todos os alarmes
// Alarme 1: A1M1=A1M2=A1M3=A1M4=1
// Alarme 2: A2M2=A2M3=A2M4=1
void rtc_alm_masc(void){
  byte x;
  for (x=0x7; x<0xE; x++)
    rtc_wr(x,0x80|rtc_rd(x));
}

// Habilitar alarme (A1IE=1 ou A2IE=1)
void rtc_ALM(byte qual){
  byte x;
  x=rtc_rd(0xE);  //Ler registrador
  if (qual==1)  x |= BIT0;
  if (qual==2)  x |= BIT1;
  x |= BIT2;      //Fazer INTCN = 1
  rtc_wr(0xE,x);  //Escrever de volta
}

// Desabilitar alarme (A1IE=0 ou A2IE=0)
void rtc_alm(byte qual){
  byte x;
  x=rtc_rd(0xE);  //Ler registrador
  if (qual==1)  x &= ~BIT0;
  if (qual==2)  x &= ~BIT1;
  rtc_wr(0xE,x);  //Escrever de volta
}

// Apaga a Flag de alarme (A1F=0 ou A2F=0)
void rtc_alm_flag(byte qual){
  byte x;
  x=rtc_rd(0xF);  //Ler registrador
  if (qual==1)  x &= ~BIT0;
  if (qual==2)  x &= ~BIT1;
  rtc_wr(0xF,x);  //Escrever de volta
}

// Habilitar modo interrupção (INTCN=1)
// Pino INT/SQW=0 se acontece alarme
// É preciso para ligar o KXC
void rtc_INTCN(void){
  byte x;
  x=rtc_rd(0xE);  //Ler registrador
  x |= BIT2;
  rtc_wr(0xE,x);  //Escrever de volta
}

// Desabilitar modo interrupção (INTCN=0)
// Pino INT/SQW=onda quadrada
// Não serve para ligar o KXC
void rtc_intcn(void){
  byte x;
  x=rtc_rd(0xE);  //Ler registrador
  x &= ~BIT2;
  rtc_wr(0xE,x);  //Escrever de volta
}

// Escrever uma data no RTC
// dd/mm/aa - 2 dígitos para cada valor
byte rtc_wr_data(byte *vt){
  byte vetor[3];
  vetor[0]=10*vt[0]+vt[1];
  vetor[1]=10*vt[3]+vt[4];
  vetor[3]=10*vt[6]+vt[7];
  rtc_wr_blk(0,vetor,3);
  return TRUE;
}

// Atualizar a data ou a hora
// dd/mm/aa --> formato da data
// hh:mm:ss --> formato da hora 
byte rtc_wr_data_hora(char *vt){
  byte x1,x2,x3;
  byte vetor[3];
  //ser_str("RTC_Data_Hora = ");
  //ser_str(vt);
  
  // Hora (hh:mm:ss)
  if (vt[2] == ':' && vt[5]==':'){
    vetor[2]=16*(vt[0]-0x30)+(vt[1]-0x30);  //Segundos
    vetor[1]=16*(vt[3]-0x30)+(vt[4]-0x30);  //Minutos
    vetor[0]=16*(vt[6]-0x30)+(vt[7]-0x30);  //Horas
    rtc_wr_blk(0,vetor,3);
    return(TRUE);
  }

  //Data (dd/mm/aa)
  if (vt[2] == '/' && vt[5]=='/'){
    vetor[0]=16*(vt[0]-0x30)+(vt[1]-0x30);  //Dia
    vetor[1]=16*(vt[3]-0x30)+(vt[4]-0x30);  //Mês
    vetor[2]=16*(vt[6]-0x30)+(vt[7]-0x30);  //Ano
    rtc_wr_blk(4,vetor,3);
    return(TRUE);
  }
  return FALSE;
}


// Retira data ou hora da fila SERI e escreve no RTC
// Já tem a certeza de que o próximo da fila é um número
// dd/mm/aa --> formato da data
// hh:mm:ss --> formato da hora 
byte rtc_wr_data_hora_velha(void){
  byte x1,x2,x3,x4,x5,hora;
  byte vet[7];
  if (seri_bcd8(&x1)==FALSE)  return FALSE;
  if (seri_tira(&x2)==FALSE)  return FALSE;
  if (seri_bcd8(&x3)==FALSE)  return FALSE;
  if (seri_tira(&x4)==FALSE)  return FALSE;
  if (seri_bcd8(&x5)==FALSE)  return FALSE;

  //ser_str("Recebeu: ");
  //ser_hex8(x1);   ser_char(x2);
  //ser_hex8(x3);   ser_char(x4);
  //ser_hex8(x5);   ser_crlf(1);
  
  if (x2 != x4) return FALSE;
  if (x2 == ':'){
    vet[2]=x1;
    vet[1]=x3;
    vet[0]=x5;
    rtc_wr_blk(0,vet,3);
    return TRUE;
  }
  if (x2 == '/'){
    vet[0]=x1;
    vet[1]=x3;
    vet[2]=x5;
    rtc_wr_blk(4,vet,3);
    return TRUE;
  }
  return FALSE;
}

// (110) Escrever num registrador do RTC
void rtc_wr(byte reg, byte dado) {
  twi_start(110);          //START
  twi_er(RTC_EWR, 111);    //Endereçar MPU para escrita
  twi_dado_er(reg, 112);   //Informar acesso ao PWR_MGMT_1 (0x6B)
  twi_dado_er(dado, 113);  //Selecionar PLL eixo X como referência
  twi_stop();              //Gerar STOP para finalizar
}

// (120) Ler um registrador do MPU
byte rtc_rd(byte reg) {
  byte dado;
  twi_start(120);                //START
  twi_er(RTC_EWR, 121);           //Endereçar MPU para escrita
  twi_dado_er(reg, 122);         //Informar registrador
  twi_start_rep(123);            //START Repetido
  twi_et(RTC_ERD, 124);           //Endereçar MPU para leitura
  dado = twi_dado_et_nack(125);  //Receber dado do MPU com NACK
  twi_stop();                    //Gerar STOP para finalizar
  return dado;
}

// (130) Escrever um bloco de dados no MPU a partir de um registrador
void rtc_wr_blk(byte reg, byte *dado, byte qtd) {
  byte i;
  twi_start(130);                //START
  twi_er(RTC_EWR, 131);          //Endereçar MPU para escrita
  twi_dado_er(reg, 132);         //Informar acesso ao PWR_MGMT_1 (0x6B)
  for (i = 0; i < qtd; i++)
    twi_dado_er(dado[i], 133);   //Selecionar PLL eixo X como referência
  twi_stop();                   //Gerar STOP para finalizar
}

// (140) Ler um bloco do MPU a partir de um registrador
void rtc_rd_blk(byte reg, byte *dado, byte qtd) {
  byte i;
  twi_start(140);                //START
  twi_er(RTC_EWR, 141);          //Endereçar MPU para escrita
  twi_dado_er(reg, 142);         //Informar registrador
  twi_start_rep(143);            //START Repetido
  twi_et(RTC_ERD, 144);          //Endereçar MPU para leitura
  for (i=0; i<qtd-1; i++)
    dado[i] = twi_dado_et_ack(145);  //Receber dados e gerar ACK
  dado[i] = twi_dado_et_nack(146);  //Receber último dado e gerar NACK
  twi_stop();                   //Gerar STOP para finalizar
}
