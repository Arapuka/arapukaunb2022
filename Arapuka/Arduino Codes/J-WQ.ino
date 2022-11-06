// KXCe (19/12/2020)
// Rotinas para acessar memo W25Q32
// PB0 = CE-WQ (flash) --> spi_config() inicializa

// Identificadores da memória
#define W25Q80    0x13
#define W25Q16    0x14
#define W25Q32    0x15
#define W25Q64    0x16  //? Confirmar número
#define W25Q128   0x17

// Constantes para a W25Q128
#define WQ_WEL_SET       0x06  //Fazer WEL=1 Write Enable
#define WQ_WEL_RST       0x04  //Fazer WEL=0 Write Disable
#define WQ_SR1_RD        0x05  //Ler Reg 1
#define WQ_SR2_RD        0x35  //Ler Reg 2
#define WQ_SR_WR         0x01  //Escrever SR1 e SR0 (Seq: D7-D0 e D8-D15)
#define WQ_DT_RD         0x03  //Ler posições da memória
#define WQ_PAGE_PROG     0x02  //A23-A0 e D7-D0
#define WQ_BLK_ERASE_64K 0xD8  //A23-A0
#define WQ_BLK_ERASE_32K 0x52  //A23-A0
#define WQ_SEC_ERASE_4K  0x20  //A23-A0
#define WQ_CHIP_ERASE    0x60  //ou 0xC7
#define WQ_CHIP_ERASE1   0xC7  //ou 0x60
#define WQ_ERASE_SUSP    0x75  //
#define WQ_ERASE_RESUME  0x7A  //
#define WQ_POWER_DWN     0xB9  //
#define WQ_MANUF_ID      0x90  //Manufacturer ID
#define WQ_UNIQUE_ID     0x4B  //Unique ID Number
#define WQ_JEDEC_ID      0x9f  //Código do fabricante

#define W25_TAM_BLK  32    //Tam bloco para gravar/ler na W25Q32
#define T64K          0x10000L    //64K = 65536

// Criar os campos para uma fermentação
// Retorna o nr da pag de 64 KB usada
// Retorna ZERO se a FLASH estiver cheia
// Sempre prepara da 'D' e 'H' (densidade a cada hora)
byte wq_dir_cria(byte cmdo, char *nome){
  byte pag;
  byte vet[7],msg[16];
  long adr;
  word i;
  
  pag=wq_dir_prox();
  if (pag==0) return 0; //Flash cheia

  // Por segurança, apagar toda pag de 64 KB a ser usada
  wq_erase_64k(pag);

  //Reg 0
  adr=(long)pag<<16;
  //ser_str("pag=");  ser_hex8(pag);
  //ser_str("  adr=");  ser_hex32(adr);
  //ser_crlf(1);
  wq_wr_word_bcd(adr+REG0_CONT,0x0000);    //0000
  //wq_wr_byte(adr+REG0_PROX,pag);
  wq_wr_str(adr+REG0_NOME,nome);

  //Reg 1
  adr += 16;
  wq_wr_word_bcd(adr+REG1_CONT,0x0001);    //0001
  rtc_rd_blk(0,vet,7);                              //Ler data e hora
  str_data_hora(vet, msg);
  msg[8]=':';
  wq_wr_blk(adr+REG1_DATA,msg,14);                  //Gravar dd/mm/aa:hh:mm

  //Reg 2
  // Copia D/T, H/M e n da EEPROM
  adr += 16;
  wq_wr_word_bcd(adr+REG2_CONT,0x0002);                 //0002
  wq_wr_byte(adr+REG2_TIPO,eeprom_rd_byte(EE_MED_DT));  //Copia "D" ou "H" da EEPROM
  wq_wr_byte(adr+REG2_FREQ,eeprom_rd_byte(EE_ALM_HM));  //Copia "H" ou "M' da EEPROM
  wq_wr_byte(adr+REG2_QDO,eeprom_rd_byte(EE_ALM_NN));   //Copiar "n" da EEPROM, qdo acordar

  //Reg 3
  adr += 16;
  wq_wr_word_bcd(adr+REG3_CONT,0x0003);    //0003

  //Reg 4
  adr += 16;
  wq_wr_word_bcd(adr+REG4_CONT,0x0004);    //0004
  
  //Reg 5
  adr += 16;
  wq_wr_word_bcd(adr+REG5_CONT,0x0005);    //0005

  //Reg 6
  adr += 16;
  wq_wr_word_bcd(adr+REG6_CONT,0x0006);    //0006
  
  //Reg 7
  adr += 16;
  wq_wr_word_bcd(adr+REG7_CONT,0x0007);    //0007

  // Gravar parâmetros na EEPROM
  adr=(long)pag<<16;      //Pag de 64KB da Flash
  adr+=REG_DADOS_INI;     //Início área dados da FLASH
  eeprom_volta_sim((word)cmdo);               //Voltar ao ligar
  eeprom_wr_word32(EE_PROX_ADR, adr);   //Adr a ser usado
  
  //EEPROM
  adr=(long)pag<<16;           //EEPROM copia
  adr+=REG_EEPROM;
  //ser_str("adr FLASH(EEPROM) = ");
  //ser_hex32(adr);
  //ser_crlf(1);
  for (i=0; i<1024; i+=16){
    eeprom_rd_seq(i,msg,16);
    wq_wr_blk(adr,msg,16);
    adr+=16;
  }
  return pag;
}


// Buscar bloco de 64KB livre
// Usado para iniciar Fermentação
// Retorna bloco de 64 KB que está livre
// Testa se os primeiros 16 bytes são iguais a 0xFF
// Retorna ZERO para indicar memória cheia
byte wq_dir_prox(void){
  byte pag,maxp;
  long  adr;
  byte vet[16];
  byte vazio;
  byte i;
  maxp=wq_tam>>16;    //Qtd de páginas de 64KB
  pag=1;
  while(pag<maxp){
    adr=(long)pag<<16;
    wq_rd_blk(adr, vet, 16);
    vazio=TRUE;
    for (i=0; i<16; i++){
      if (vet[i] != 0xFF) vazio=FALSE;
    }
    if (vazio==TRUE)  return pag;
    pag++;
  }
  //ser_str("Flash Cheia");
  return 0;   //Flash Cheia
}

/*
// Preparar para iniciar gravação de dados
// nr = número do diretório
void wq_reg_prep(byte nr){
  long adr;
  char vet[16],msg[16];
  adr=nr*( (long)REG_TAM*DIR_REG_QTD);  //FLASH: Início da área de dados
  wq_erase_64k(adr);

  // Reg0
  wq_wr_word(adr+REG0_CONT,0x0000);                 //0000
  eeprom_rd_seq(EE_IDT,vet,6);                      //Ler IDT e versão
  wq_wr_blk(adr+REG0_IDT,vet,6);                    //Gravar IDT e versão
  wq_wr_byte(adr+REG0_ACEL_ESC,mpu_rd_esc_acel());  //Gravar Escala Acel
  wq_wr_byte(adr+REG0_GIRO_ESC,mpu_rd_esc_giro());  //Gravar Escala Acel

  // Reg1
  adr+=REG_TAM;
  wq_wr_word(adr+REG1_CONT,0x0001);                 //0001
  rtc_rd_blk(0,vet,7);                              //Ler data e hora
  str_data_hora(vet, msg);
  msg[8]=':';
  wq_wr_blk(adr+REG1_DATA,msg,14);                  //Gravar dd/mm/aa:hh:mm

  // Reg2
  adr+=REG_TAM;
  wq_wr_word(adr+REG2_CONT,0x0002);                 //0002
  eeprom_rd_seq(EE_OFF_AX,msg,12);                  //Ler Off Set
  wq_wr_blk(adr+REG2_OFF_AX,msg,12);                //Gravar Off Set
  
  // Reg3
  adr+=REG_TAM;
  wq_wr_word(adr+REG3_CONT,0x0003);                 //0003
  eeprom_rd_seq(EE_AX_VERT,msg,12);                 //Vert e Cal
  wq_wr_blk(adr+REG3_AX_VERT,msg,12);               //Gravar Vert e Cal

}
*/

// Retorna tamanho da memória em MB
byte wq_ser_tam(byte id){
  switch(id){
    case W25Q32:  return 4;
    case W25Q64:  return 8;;
    case W25Q128: return 16;
    default:      return 0;
  }
}

// Imprime identificação da memória Flash
void wq_ser_id(byte id){
  switch(id){
    case W25Q32:  ser_str("W25Q32");  break;
    case W25Q64:  ser_str("W25Q64");  break;
    case W25Q128: ser_str("W25Q128"); break;
    default:      ser_str("W?");      break;
  }
}

// Ler um valor de 16 bits da FLASH
// Big Endian
word wq_rd_word(long adr){
  word x;
  while(w25_ocupado()==TRUE) ser_char('*');
  wq_ce();   //Selecionar SRAM
  spi_transf(WQ_DT_RD);
  spi_transf((adr>>16)&0xFF);
  spi_transf((adr>>8)&0xFF);
  spi_transf(adr&0xFF);
  x=spi_transf(0);
  x=(x<<8)+spi_transf(0);
  wq_CE();      //De-selecionar SRAM
  return x;
}

// Ler um valor de 8 bits da FLASH
byte wq_rd_byte(long adr){
  byte x;
  while(w25_ocupado()==TRUE) ser_char('*');
  wq_ce();   //Selecionar SRAM
  spi_transf(WQ_DT_RD);
  spi_transf((adr>>16)&0xFF);
  spi_transf((adr>>8)&0xFF);
  spi_transf(adr&0xFF);
  x=spi_transf(0);
  wq_CE();      //De-selecionar SRAM
  return x;
}

// Escrever uma word em BCD na FLASH
// Converte o "dado" em BCD. Big Endian
void wq_wr_word_bcd(long adr, word dado){
  byte x,y;
  char msg[7];
  str_dec16u(dado, msg);    //Gerar string
  while(wq_ocupado()==TRUE) ser_char('*');
  wq_WEL();
  wq_ce();   //Selecionar Flash
  spi_transf(WQ_PAGE_PROG);
  spi_transf(adr>>16);
  spi_transf(adr>>8);
  spi_transf(adr);
  x=msg[1]-'0';
  x=(x<<4)+(msg[2]-'0');
  spi_transf(x);        //Escrever MSByte
  x=msg[3]-'0';
  x=(x<<4)+(msg[4]-'0');
  spi_transf(x);    //Escrever LSByte
  wq_CE();      //De-selecionar Flash
  wq_wel();
}


// Escrever um valor de 16 bits na FLASH
// Big Endian
void wq_wr_word(long adr, word dado){
  while(wq_ocupado()==TRUE) ser_char('*');
  wq_WEL();
  wq_ce();   //Selecionar Flash
  spi_transf(WQ_PAGE_PROG);
  spi_transf(adr>>16);
  spi_transf(adr>>8);
  spi_transf(adr);
  spi_transf(dado>>8); //Escrever MSByte
  spi_transf(dado);    //Escrever LSByte
  wq_CE();      //De-selecionar Flash
  wq_wel();
}

// Escrever um valor de 8 bits na FLASH
void wq_wr_byte(long adr, byte dado){
  while(wq_ocupado()==TRUE) ser_char('*');
  wq_WEL();
  wq_ce();   //Selecionar Flash
  spi_transf(WQ_PAGE_PROG);
  spi_transf(adr>>16);
  spi_transf(adr>>8);
  spi_transf(adr);
  spi_transf(dado); //Escrever byte
  wq_CE();      //De-selecionar Flash
  wq_wel();
}

// Escrever uma string terminada com 0 na WQ
// Deve estar dentro de uma página de 256 bytes
// Ao terminar WEL=0
void wq_wr_str(long adr, byte *vet){
  int i;
  while(wq_ocupado()==TRUE) ser_char('*');
  wq_WEL();
  wq_ce();   //Selecionar SRAM
  spi_transf(WQ_PAGE_PROG);
  spi_transf(adr>>16);
  spi_transf(adr>>8);
  spi_transf(adr);
  i=0;
  while (vet[i] != '\0')
    spi_transf(vet[i++]);
  spi_transf(vet[i++]);   //Escrever o ZERO
  wq_CE();      //De-selecionar Flash
  wq_wel();
}

// Ler da Flash uma string terminada em 0
// Qtd é o tamanho do vetor
void wq_rd_str(long adr, byte *vet, word qtd){
  word i;
  while(w25_ocupado()==TRUE) ser_char('*');
  wq_ce();   //Selecionar SRAM
  spi_transf(WQ_DT_RD);
  spi_transf((adr>>16)&0xFF);
  spi_transf((adr>>8)&0xFF);
  spi_transf(adr&0xFF);
  i=0;
  while(i<qtd){ 
    vet[i]=spi_transf(0);
    if (vet[i]=='\0') break;
    i++;
  }
  if (i==qtd) vet[i-1]='\0';  //Saiu pelo limite qtd, marcar fim string
  wq_CE();      //De-selecionar SRAM
}



// Escrever um bloco na WQ
// Deve estar dentro de uma página de 256 bytes
// Ao terminar WEL=0
void wq_wr_blk(long adr, byte *vet, word qtd){
  int i;
  while(wq_ocupado()==TRUE) ser_char('*');
  wq_WEL();
  wq_ce();   //Selecionar SRAM
  spi_transf(WQ_PAGE_PROG);
  spi_transf(adr>>16);
  spi_transf(adr>>8);
  spi_transf(adr);
  for (i=0; i<qtd; i++){
    spi_transf(vet[i]);
  }
  wq_CE();      //De-selecionar Flash
  wq_wel();
}

// Escrever uma página completa (256 bytes) da WQ
// Ao terminar WEL=0
void wq_wr_pag(word pag, byte *vet){
  int i;
  while(wq_ocupado()==TRUE) ser_char('*');
  wq_WEL();
  wq_ce();   //Selecionar SRAM
  spi_transf(WQ_PAGE_PROG);
  spi_transf(pag>>8);
  spi_transf(pag);
  spi_transf(0);    //Múltiplo de 256
  for (i=0; i<256; i++){
    spi_transf(vet[i]);
  }
  wq_CE();      //De-selecionar Flash
  wq_wel();
}

// Ler uma certa quantidade de posições
void wq_rd_blk(long adr, byte *vet, long qtd){
  long i;
  //while(w25_ocupado()==TRUE) ser_char('*');
  wq_ce();   //Selecionar SRAM
  spi_transf(WQ_DT_RD);
  spi_transf((adr>>16)&0xFF);
  spi_transf((adr>>8)&0xFF);
  spi_transf(adr&0xFF);
  for (i=0; i<qtd; i++){
    //delay(1000);
    vet[i]=spi_transf(0);
    //ser_dec32unz(i);
    //ser_spc(1);
    //ser_hex8(vet[i]);
    //ser_spc(1);
    //ser_crlf(1);
  }  
  wq_CE();      //De-selecionar SRAM
}



// Ler uma página completa (256 Bytes) da Flash
void wq_rd_pag(word pag, byte *vet){
  word i;
  while(wq_ocupado()==TRUE) ser_char('*');
  wq_ce();   //Selecionar SRAM
  spi_transf(WQ_DT_RD);
  spi_transf(pag>>8);
  spi_transf(pag);
  spi_transf(0);    //Múltiplo de 256
  for (i=0; i<256; i++){
    vet[i]=spi_transf(0);
  }  
  wq_CE();      //De-selecionar SRAM
}

// Apagar toda a memória - Demora!
void wq_erase_chip(void){
  int i=0;
  ser_str("\nErase Chip");
  ser_crlf(1);
  wq_WEL();
  wq_ce();      //Selecionar Flash
  spi_transf(WQ_CHIP_ERASE); //0x60
  wq_CE();      //De-selecionar SRAM
  while(TRUE){
    if (wq_ocupado()==FALSE)  break;
    if (flag_seg==TRUE){
      flag_seg=FALSE;
      ser_dec32unz(++i);
      ser_spc(1);
      if ((i%10)==0)  ser_crlf(1);
    }
  }
  ser_str("\nPronto\n");
}

// Apagar (0xFF) um bloco de 64KB
void wq_erase_64k(byte pag){
  long i=0;
  //ser_str("\nErase_64K = ");
  //ser_hex32(adr);
  //ser_crlf(1);
  wq_WEL();
  wq_ce();   //Selecionar Flash
  spi_transf(WQ_BLK_ERASE_64K); //0xAA 0000
  spi_transf(pag);
  spi_transf(0);
  spi_transf(0);
  wq_CE();      //De-selecionar SRAM
  while (wq_ocupado()==TRUE){
    espera_100ms(2);
    //ser_dec32unz(i++);
    //ser_spc(1);
    ser_char('.');
  }
//  ser_str("\nPronto\n");
}

// Apagar (0xFF) um setor de 4 KB
// 1111 1111   1111 0000   0000 0000
// 4095 setores de 4 KB
// Ao terminar WEL=0 automaticamente
void wq_erase_4k(word sec){
  ser_str("Erase_4K = ");
  ser_dec16(sec);
  ser_crlf(1);
  wq_WEL();
  wq_ce();   //Selecionar SRAM
  spi_transf(WQ_SEC_ERASE_4K);
  spi_transf( (sec>>8) & 0xFF );
  spi_transf( (sec&0xF) << 4);
  spi_transf(0);
  wq_CE();      //De-selecionar SRAM
}

// WQ está ocupado?
// Ocupado=TRUE Livre=FALSE
byte wq_ocupado(void){
  byte x;
  wq_ce();   //Selecionar SRAM
  spi_transf(WQ_SR1_RD);
  x=spi_transf(0);
  wq_CE();      //De-selecionar SRAM
  if ((x&1)==0)  return FALSE;
  else           return TRUE;
}

// WEL = 0 --> Write Enable = 0
// Bloqueia as escritas
void wq_wel(void){
    wq_ce();   //Selecionar SRAM
    spi_transf(WQ_WEL_RST);     
    wq_CE();      //De-selecionar SRAM
}

// WEL = 1 --> Write Enable = 0
// Permite as escritas
void wq_WEL(void){
    wq_ce();   //Selecionar SRAM
    spi_transf(WQ_WEL_SET);     
    wq_CE();      //De-selecionar SRAM
}

// Ler Reg de Status 1
byte wq_rd_sr1(void){
  byte d;
  wq_ce();      //Selecionar Flash
  spi_transf(WQ_SR1_RD);
  d=spi_transf(0);
  wq_CE();      //De-selecionar Flash
  return d;
}

// Ler Reg de Status 2
byte wq_rd_sr2(void){
  byte d;
  wq_ce();      //Selecionar Flash
  spi_transf(WQ_SR2_RD);
  d=spi_transf(0);
  wq_CE();      //De-selecionar Flash
  return d;
}

// Retorna
// vt[0] = Manufacturer ID
// vt[1] = Device ID
void wq_manuf_dev_id(byte *vt){
  wq_ce();   //Selecionar SRAM
  spi_transf(WQ_MANUF_ID);     //JEDEC
  spi_transf(0); 
  spi_transf(0); 
  spi_transf(0);
  vt[0]=spi_transf(0);
  vt[1]=spi_transf(0);
  wq_CE();      //De-selecionar FLASH
}

// Testar JEDEC para verificar se W25Q32 responde
int wq_jedec_id(byte *vt){
  wq_ce();   //Selecionar FLASH
  spi_transf(WQ_JEDEC_ID);     //JEDEC
  vt[0]=spi_transf(0); 
  vt[1]=spi_transf(0); 
  vt[2]=spi_transf(0);
  wq_CE();      //De-selecionar FLASH
}

void wq_unique_id(byte *vt){
  byte i;
  wq_ce();   //Selecionar FLASH
  spi_transf(WQ_UNIQUE_ID);   //UNIQUE ID NUMBER
  spi_transf(0);              //Dummy 1
  spi_transf(0);              //Dummy 2
  spi_transf(0);              //Dummy 3
  spi_transf(0);              //Dummy 4
  for (i=0; i<8; i++)   vt[i]=spi_transf(0); 
  wq_CE();      //De-selecionar FLASH
}

// CE-WQ controle   (PB0 = CE-WQ)
void wq_ce(void) {noInterrupts();  PORTB&=~(1<<PB0);} //CE-WQ=0 
void wq_CE(void) {PORTB|=(1<<PB0); interrupts();    } //CE-WQ=1 
void wq_Ce(void) {PORTB^=(1<<PB0);}     //Inverter CE (Ociloscópio) 

////////////////////////////////////////////////////////////////


// Verificar se um bloco de 64K está apagado
char w25_check_FF(long adr){
  long i;
  char apagado;
  unsigned char x;
  char msg[100];
  //Serial.print("\nCheck\n");
  while(w25_ocupado()==TRUE)  ; //Serial.print("*");
  wq_ce();   //Selecionar SRAM
  spi_transf(WQ_DT_RD);
  spi_transf(adr>>16);
  spi_transf(adr>>8);
  spi_transf(adr);
  apagado=TRUE;
  for (i=0; i<T64K; i++){
    x=spi_transf(0);
    if (x!=0xFF){
      apagado=FALSE;
      //sprintf(msg,"Erro: lido=%X e i=%d\n",x);
      //Serial.print(msg);
      break;
    }
  }  
  wq_CE();      //De-selecionar SRAM
  return apagado;
}






/*
// Inicializar W25Q32 e SPI
void wq_inic(void){
  pinMode(MOSI,OUTPUT);   //Configurar MOSI como saída
  pinMode(MISO,INPUT_PULLUP);
  pinMode(SCK, OUTPUT);   //Configurar SCK como saída
  pinMode(CS1,  OUTPUT);   //Configurar pino #CS1
  digitalWrite(CS1,HIGH);  //Iniciar com #CS1=High
  pinMode(CS2,  OUTPUT);   //Configurar pino #CS2
  digitalWrite(CS2,HIGH);  //Iniciar com #CS2=High
  pinMode(SS,  OUTPUT);   //Configurar pino SS (memo SD)
  digitalWrite(SS,HIGH);  //Iniciar com SS=High (memo SD)
  // Incializar porta SPI
  SPCR = (1<<SPE) | (1<<MSTR); //Hab. SPI Mestre, 4 MHz
}
*/

// Testar JEDEC para verificar se W25Q32 responde
int w25_jedec_teste(void){
  unsigned char d1,d2,d3;
  char msg[100];
  // Primeiro JEDEC teste dá erro! Não sei por que.
  // Esse é ignorado
  wq_ce();   //Selecionar SRAM
  spi_transf(WQ_JEDEC_ID);     //JEDEC
  d1=spi_transf(0); 
  d2=spi_transf(0); 
  d3=spi_transf(0);
  wq_CE();      //De-selecionar SRAM
  // Esse é o segundo JEDEC teste
  // Usado para tomar decisão
  wq_ce();   //Selecionar SRAM
  spi_transf(WQ_JEDEC_ID);     //JEDEC
  d1=spi_transf(0); 
  d2=spi_transf(0); 
  d3=spi_transf(0);
  wq_CE();      //De-selecionar SRAM
  ser_str("JEDEC: ");
  ser_hex8(d1); ser_spc(1);
  ser_hex8(d2); ser_spc(1);
  ser_hex8(d3); ser_spc(1);
  /*
  if (d1==0xEF && d2==0x40 && d3==0x16) return TRUE;
  else{
    ser_str("ERRO W25 JEDEC=EF-40-16");
    return FALSE;
  }
  */
}



// Apagar (0xFF) toda a memória
// Ao terminar WEL=0
void w25_chip_erase(void){
  wq_ce();   //Selecionar SRAM
  spi_transf(WQ_CHIP_ERASE);
  wq_CE();      //De-selecionar SRAM
}

// Apagar (0xFF) um setor de 64 KB
// Ao terminar WEL=0
void w25_blk_erase_64k(long adr){
  while(w25_ocupado()==TRUE)  ser_char('!');
  w25_WEL();
  wq_ce();   //Selecionar SRAM
  spi_transf(WQ_BLK_ERASE_64K);
  spi_transf(adr>>16);
  spi_transf(adr>>8);
  spi_transf(adr);
  wq_CE();      //De-selecionar SRAM
}

// Apagar (0xFF) um setor de 32 KB
// Ao terminar WEL=0
void w25_blk_erase_32k(long adr){
  wq_ce();   //Selecionar SRAM
  spi_transf(WQ_BLK_ERASE_32K);
  spi_transf(adr>>16);
  spi_transf(adr>>8);
  spi_transf(adr);
  wq_CE();      //De-selecionar SRAM
}

// Apagar (0xFF) um setor de 4 KB
// Ao terminar WEL=0
void w25_sec_erase_4k(long adr){
  wq_ce();   //Selecionar SRAM
  spi_transf(WQ_SEC_ERASE_4K);
  spi_transf(adr>>16);
  spi_transf(adr>>8);
  spi_transf(adr);
  wq_CE();      //De-selecionar SRAM
}

// Escrever um bloco na W25
// Cuida das fronteiras de página
// Ao terminar WEL=0
void w25_wr_blk(long adr, byte *vet, long qtd){
  int i=0;
  while(qtd!=0){
    while(w25_ocupado()==TRUE) ser_char('*');
    w25_WEL();
    wq_ce();   //Selecionar SRAM
    spi_transf(WQ_PAGE_PROG);
    spi_transf(adr>>16);
    spi_transf(adr>>8);
    spi_transf(adr);
    delay(100);
    while(qtd!=0){
      spi_transf(vet[i++]);
      qtd--;
      adr++;
      if ((adr&0xff)== 0) break;
    }
    wq_CE();      //De-selecionar SRAM
  }
}

// Escrever uma página (1 até 256 bytes)
// Note que existem as fronteiras de página
// Ao atingir a fronteira, volta para o início da página
// Usar potências de 2: 8, 16, 32
// Ao terminar WEL=0
void w25_wr_pag(long adr, byte *vet, long qtd){
  int i;
  
  char msg[200];
  ser_str("\nGravar:");
  for (i=0; i<qtd; i++){
    //sprintf(&msg[i],"%c",vet[i]);
  }
  msg[i]='\0';
  //Serial.println(msg);
  
  while(w25_ocupado()==TRUE)  ser_char('*');
  w25_WEL();
  wq_ce();   //Selecionar SRAM
  spi_transf(WQ_PAGE_PROG);
  spi_transf(adr>>16);
  spi_transf(adr>>8);
  spi_transf(adr);
  delay(100);
  for (i=0; i<qtd; i++){
    //Serial.print(vet[i],HEX);
    //Serial.print(" ");
    //delay(5000);
    spi_transf(vet[i]);
    //spi_transf(0x3B);
  }
  wq_CE();      //De-selecionar SRAM
}

// Ler uma certa quantidade de posições
void w25_rd_blk(long adr, byte *vet, long qtd){
  long i;
  //while(w25_ocupado()==TRUE) ser_char('*');
  wq_ce();   //Selecionar SRAM
  spi_transf(WQ_DT_RD);
  spi_transf(adr>>16);
  spi_transf(adr>>8);
  spi_transf(adr);
  for (i=0; i<qtd; i++){
    //delay(1000);
    vet[i]=spi_transf(0);
    //Serial.print(vet[i],HEX);
    //Serial.print(" ");
  }  
  wq_CE();      //De-selecionar SRAM
}

// W25 está ocupado?
// Ocupado=TRUE Livre=FALSE
byte w25_ocupado(void){
  byte x;
  wq_ce();   //Selecionar SRAM
  spi_transf(WQ_SR1_RD);
  x=spi_transf(0);
  wq_CE();      //De-selecionar SRAM
  if ((x&1)==0)  return FALSE;
  else           return TRUE;
}

// Escrever no Reg de Status (1 e 2)
void w25_sr_wr(int dado){
  wq_ce();   //Selecionar SRAM
  spi_transf(WQ_SR_WR);
  spi_transf(dado>>8);
  spi_transf(dado);
  wq_CE();      //De-selecionar SRAM
}

// Ler Reg de Status 2
byte w25_sr2_rd(void){
  byte d;
  char msg[50];
  wq_ce();   //Selecionar SRAM
  spi_transf(WQ_SR2_RD);
  d=spi_transf(0);
  wq_CE();      //De-selecionar SRAM
  return d;
}


// Ler Reg de Status 1
byte w25_sr1_rd(void){
  byte d;
  char msg[50];
  wq_ce();   //Selecionar SRAM
  spi_transf(WQ_SR1_RD);
  d=spi_transf(0);
  wq_CE();      //De-selecionar SRAM
  return d;
}

// WEL = 0 --> Write Enable = 0
// Bloqueia as escritas
void w25_wel(void){
    wq_ce();   //Selecionar SRAM
    spi_transf(WQ_WEL_RST);     
    wq_CE();      //De-selecionar SRAM
}

// WEL = 1 --> Write Enable = 0
// Permite as escritas
void w25_WEL(void){
    wq_ce();   //Selecionar SRAM
    spi_transf(WQ_WEL_SET);     
    wq_CE();      //De-selecionar SRAM
}

// Retorna
// vt[0] = Manufacturer ID
// vt[1] = Device ID
void w25_manuf_dev_id(byte *vt){
  byte d1,d2;
  wq_ce();   //Selecionar SRAM
  spi_transf(WQ_MANUF_ID);     //JEDEC
  spi_transf(0); 
  spi_transf(0); 
  spi_transf(0);
  d1=spi_transf(0);
  d2=spi_transf(0);
  wq_CE();      //De-selecionar SRAM
  //sprintf(msg,"MANUFACTURER ID: %02X DEVICE ID: %02X",d1,d2);
  //Serial.println(msg);
}
