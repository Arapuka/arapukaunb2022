// GPS - Rotinas para acesso
// CXP - Caixa Preta
// 22/01/2019

// GPS: Habilitar recepção e interrupção (RX3)
void gps_int(void){
  UCSR3B = (1<<RXCIE3) | (1<<RXEN3);  //RXIE=1, RXEN=1 
}

// GPS: Desabilitar recepção e interrupção (RX3)
void gps_des_int(void){
  UCSR3B &= ~((1<<RXCIE3) | (1<<RXEN3));  //RXIE=0, RXEN=0
}

// Extrair dados do GPS
// Identifica o tipo de mensagem e extrai os dados de interesse
// Atualiza o vetor gps_dados
void gps_extrai(byte *vt){
  byte tipo,i=0;
  tipo = gps_idtf(vt);
  i=0;
  switch(tipo){
    case GPS_RMC:
      i=gps_come_vg(i,vt,1);     gps_cpy_vg(&vt[i],&gps_dados[GPS_HORA]);     //Hora
      i=gps_come_vg(i,vt,1);     gps_cpy_vg(&vt[i],&gps_dados[GPS_STATUS]);   //Status
      i=gps_come_vg(i,vt,1);     gps_cpy_vg(&vt[i],&gps_dados[GPS_LAT]);      //Latitude
      i=gps_come_vg(i,vt,1);     gps_cpy_vg(&vt[i],&gps_dados[GPS_NS]);       //N S
      i=gps_come_vg(i,vt,1);     gps_cpy_vg(&vt[i],&gps_dados[GPS_LONG]);     //Longitude
      i=gps_come_vg(i,vt,1);     gps_cpy_vg(&vt[i],&gps_dados[GPS_EW]);       //E W
      i=gps_come_vg(i,vt,1);     gps_cpy_vg(&vt[i],&gps_dados[GPS_VEL_NOS]);  //Veloc nos
      i=gps_come_vg(i,vt,1);     gps_cpy_vg(&vt[i],&gps_dados[GPS_CURSO]);    //Curso
      i=gps_come_vg(i,vt,1);     gps_cpy_vg(&vt[i],&gps_dados[GPS_DATA]);     //Data
      gps_tipo=GPS_RMC;           //Marcar que recebeu RMC
      break;

    case GPS_VTG:
      i=gps_come_vg(i,vt,7);     gps_cpy_vg(&vt[i],&gps_dados[GPS_VEL_KPH]);  //VEloc kph
      i=gps_come_vg(i,vt,1);     gps_cpy_vg(&vt[i],&gps_dados[GPS_VEL_UN]);   //Unidade Veloc (kph)
      gps_tipo=GPS_VTG;           //Marcar que recebeu VTG
      break;

    case GPS_GGA:
      i=gps_come_vg(i,vt,7);     gps_cpy_vg(&vt[i],&gps_dados[GPS_QTD_SAT]);  //Quantidade de satélites
      i=gps_come_vg(i,vt,2);     gps_cpy_vg(&vt[i],&gps_dados[GPS_ALT]);      //Altitude nívem médio dos mares
      i=gps_come_vg(i,vt,1);     gps_cpy_vg(&vt[i],&gps_dados[GPS_ALT_UN]);   //Unidade Altitude 
      gps_tipo=GPS_GGA;           //Marcar que recebeu GGA
      break;

    case GPS_GSA:
      i=gps_come_vg(i,vt,15);    gps_cpy_vg(&vt[i],&gps_dados[GPS_PDOP]);     //PDOP
      i=gps_come_vg(i,vt,1);     gps_cpy_vg(&vt[i],&gps_dados[GPS_HDOP]);     //HDOP
      i=gps_come_vg(i,vt,1);     gps_cpy_vg(&vt[i],&gps_dados[GPS_VDOP]);     //VDOP
      gps_tipo=GPS_GSA;           //Marcar que recebeu GSA
      break;

    case GPS_GSV:
      gps_tipo=GPS_GSV;           //Marcar que recebeu GSV
      break;

    case GPS_GLL:
      gps_tipo=GPS_GLL;           //Marcar que recebeu GLL
      break;
  }
}

// Avançar uma certa qtd de vírgulas
byte gps_come_vg(byte ini, byte *vt, byte qtd){
  byte cont;
  byte ix=ini;
  cont=0;
  while (cont<qtd){
    while( vt[ix] != ','){
      ix++;
    }
    ix++;
    cont++;
  }
  //ix++;
  return ix;
}

// Copiar trecho de uma string para outra
// Para quando encontra uma vírgula ou "*"
// Coloca um zero no final do destinos
void gps_cpy_vg(byte *fonte, byte *dest){
  byte iy=0;
  while(fonte[iy] != ',' && fonte[iy] != '*' ){
    dest[iy]=fonte[iy];
    iy++;
  }
  dest[iy]='\0';            //Colocar 0 no final
}

// Identificar tipo de mensagens que está no vetor
// Retorna número correspondente
// 0=GPS_NADA  1=GPS_RMC  2=GPS_VTG   3=GPS_GGA
// 4=GPS_GSA   5=GPS_GSV  6=GPS_GLL
byte gps_idtf(byte *vt){
  byte letra;
  byte st=0,i=0;
  while(i<6){
    letra=vt[i++];
    switch(st){
      case  0: if (letra=='$')    {st=1;  break;}
               return GPS_NADA;
  
      case  1: if (letra=='G')    {st=2;  break;}
               return GPS_NADA;
      
      case  2: if (letra=='P')    {st=3;  break;}
               return GPS_NADA;
      
      case  3: if (letra=='R')     {st=4;  break;}
               else if(letra=='V') {st=6;  break;}
               else if(letra=='G') {st=8;  break;}
               return GPS_NADA;
               
      case  4: if (letra=='M')     {st=5;  break;}
               return GPS_NADA;
      
      case  5: if (letra=='C') return GPS_RMC;  //<== RMC
               return GPS_NADA;
  
      case  6: if (letra=='T')     {st=7;  break;}
               return GPS_NADA;
  
      case  7: if (letra=='G') return GPS_VTG;  //<== VTG
               return GPS_NADA;
  
      case  8: if (letra=='L')     {st=9;  break;}
               else if(letra=='G') {st=10; break;}
               else if(letra=='S') {st=11; break;}
               return GPS_NADA;
      
      case  9: if (letra=='L') return GPS_GLL;  //<== GLL
               return GPS_NADA;
  
      case 10: if (letra=='A') return GPS_GGA;  //<== GGA
               return GPS_NADA;
               
      case 11: if (letra=='A')     return GPS_GSA;    //<== GSA
               else if(letra=='V') return GPS_GSV;    //<== GSV
               return GPS_NADA;
    }
  }
  return GPS_NADA;
}

// Enviar msg para GPS
void gps_str(byte *msg){
  int i=0;
  while( msg[i] != '\0')
    ser_char(msg[i++]);
}

// Enviar um char para o GPS
// Sem usar interrupção
void gps_char(byte dt){
  UDR3=dt;
  while( (UCSR3A&(1<<TXC3)) == 0);  //Esperar flag tx
  UCSR3A |= 1<<TXC3;                //Apagar flag tx
}

////////////////////////////////////////////////////
/////////////// Configuração ///////////////////////
////////////////////////////////////////////////////

// Configurar porta serial 3
// Não habilita TX e nem RX
// Não habilita interrupções
void gps_config(long br){
  word dv;
  dv=(2000000L/br)-1;
  UBRR3H = dv>>8;     //MSB
  UBRR3L = dv&0xFF;   //LSB
  // Configurar 8 bits, sem paridade, 1 stop
  UCSR3A = (1<<TXC3) | (1<<U2X3);
  UCSR3B = 0;     //desab RX e TX
//  UCSR3B = (1<<RXCIE3) | (1<<TXCIE3) | (1<<RXEN3) | (1<<TXEN3);  //RXIE=TXIE=1, hab TX e RX
//  UCSR3B = (1<<RXCIE3) | (1<<TXCIE3) | (1<<TXEN3);                 //RXIE=TXIE=1, hab TX e não RX
  UCSR3C = (1<<UCSZ31)| (1<<UCSZ30);
  gps_tx_ok=TRUE;        //Já pode começar a transmitir
}

// TX3: Interrupção por dado enviado
ISR(USART3_TX_vect){
  gps_tx_ok=TRUE;  //Marcar que já transmitiu
}

// RX3: Interrupção por dado recebido
ISR(USART3_RX_vect){
  byte x;
  x=UDR3;
  if (gps_msg_fase == 0){
    gps_msg_0[gps_msg_ix++]=x;
    if (x == '\n'){
      gps_msg_0[gps_msg_ix-2]='\0';
      gps_msg_fase=1;
      gps_msg_ix=0;
      gps_msg_ok=TRUE;
    }
  }
  else{
    gps_msg_1[gps_msg_ix++]=x;
    if (x == '\n'){
      gps_msg_1[gps_msg_ix-2]='\0';
      gps_msg_fase=0;
      gps_msg_ix=0;
      gps_msg_ok=TRUE;
    }
  }
  gps_rx_ok=TRUE;   //Marcar que recebeu
}
