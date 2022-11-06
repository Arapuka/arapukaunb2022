// Puka - Modos.c

#include <msp430.h>
#include "Modos.h"
#include "Gpio.h"
#include "Timer.h"
#include "Serial.h"
#include "Lcd.h"
#include "Mpu.h"
#include "Rtc.h"
#include "Wq.h"
#include "Gps.h"
#include "Strings.h"

//10 - SW - Testar as chaves
char modo_sw  (char modo) {
    char sw1_ct, sw2_ct;    //Contar acionamentos das chaves
    sw1=sw2=FALSE;          //Cancelar acionamentos anteriores
    modo_ser1(modo);
    ser1_str(" Testar SW\n");
    lcdb_str(1,1,"SW");
    lcdb_str(2,1,"SW1=A  SW2=A");
    sw1_ct=sw2_ct=0;
    seri_config();    //Apagar fila de entrada
    ser1_str("SW1=");        ser1_dec8u(sw1_ct);
    ser1_str("  SW2=");      ser1_dec8u(sw2_ct);
    ser1_crlf(1);


    while(TRUE){
        if (seri_cmdo_x()==TRUE)    return modo;

        if (sw1_estado==ABERTA)  lcdb_char(2,5,'A');
        else                     lcdb_char(2,5,'F');
        if (sw2_estado==ABERTA)  lcdb_char(2,12,'A');
        else                     lcdb_char(2,12,'F');

        // SW1
        if (sw1==TRUE){
            sw1=FALSE;
            sw1_ct++;
            ser1_str("SW1=");
            ser1_dec8u(sw1_ct);
            ser1_crlf(1);
        }

        //SW2
        if (sw2==TRUE){
            sw2=FALSE;
            sw2_ct++;
            ser1_str("SW2=");
            ser1_dec8u(sw2_ct);
            ser1_crlf(1);
        }
    }
    return modo;
}

//11 - LEDs
// Testar Leds
// Digitar: VM, vn, Vm, VD, vd, Vd
char modo_leds  (char modo) {
    char qtd,argc[4],argv[10];
    modo_ser1(modo);
    ser1_str(" Testar LEDs\n");
    lcdb_str(1,1,"Leds");
    led_vm();
    led_vd();
    while(TRUE){
        qtd=seri_cmdo(argc,argv,10);
        if(qtd>0){
            if (argv[0]=='X' || argv[0]=='x'){
                led_vm();
                led_vd();
                return modo;
            }
            if      (str_cmp(&argv[0], "VM")) led_VM();
            else if (str_cmp(&argv[0], "vm")) led_vm();
            else if (str_cmp(&argv[0], "Vm")) led_Vm();
            else if (str_cmp(&argv[0], "VD")) led_VD();
            else if (str_cmp(&argv[0], "vd")) led_vd();
            else if (str_cmp(&argv[0], "Vd")) led_Vd();
            else                             ser1_char('?');
        }
    }
    return modo;
}


//12 - LCD
char modo_lcd  (char modo){
    char letra=0,base=0;
    char lin,col;
    modo_ser1(modo);
    while(TRUE){
        if (seri_cmdo_x()==TRUE)    return modo;
        for (lin=1; lin<3; lin++){
            for(col=1; col<17; col ++){
                letra=base+(lin-1)*16+(col-1);
                lcdb_char(lin,col,letra);
            }
        }
        delay_10ms(50);
        base++;
    }
    return modo;
}

//13 - LCD
char modo_mpu  (char modo) {
    char x;
    char vetor[14];
    int ax,ay,az,tp,gx,gy,gz;
    modo_ser1(modo);
    ser1_str(" MPU =");
    lcdb_str(1,1,"MPU");

    x=i2c_teste_adr(MPU_ADR);
    if (x==TRUE)    ser1_str(" OK");
    else{
        ser1_str(" NOK");
        return modo;
    }
    ser1_crlf(1);
    while(TRUE){
        if (seri_cmdo_x()==TRUE)    return modo;
        mpu_rd_vet(ACCEL_XOUT_H, vetor, 14);    //Ler 14 regs
        ax=vetor[ 0];    ax=(ax<<8)+vetor[ 1];
        ay=vetor[ 2];    ay=(ay<<8)+vetor[ 3];
        az=vetor[ 4];    az=(az<<8)+vetor[ 5];
        tp=vetor[ 6];    tp=(tp<<8)+vetor[ 7];
        gx=vetor[ 8];    gx=(gx<<8)+vetor[ 9];
        gy=vetor[10];    gy=(gy<<8)+vetor[11];
        gz=vetor[12];    gz=(gz<<8)+vetor[13];
        ser1_dec16(ax); ser1_spc(1);
        ser1_dec16(ay); ser1_spc(1);
        ser1_dec16(az); ser1_spc(3);
        ser1_dec16(gx); ser1_spc(1);
        ser1_dec16(gy); ser1_spc(1);
        ser1_dec16(gz); ser1_spc(1);
        ser1_crlf(1);
        lcdb_hex16(1,1, ax);
        lcdb_hex16(1,6, ay);
        lcdb_hex16(1,11,az);
        lcdb_hex16(2,1, gx);
        lcdb_hex16(2,6, gy);
        lcdb_hex16(2,11,gz);
        delay_10ms(100);
    }
    return modo;
}

//14 - RTC - Testar o RTC
char modo_rtc  (char modo) {
    char qtd,argc[4], argv[15]; //Receber comandos
    char vetor[7];
    modo_ser1(modo);
    ser1_str(" Testar RTC\n");
    lcdb_str(1,1,"RTC");
    while(TRUE){
        qtd=seri_cmdo(argc,argv,15);  //Esperar comando
        if (qtd!=0){
            ser1_str("Chegou: ");
            ser1_str(&argv[0]);
            ser1_crlf(1);

            if (argv[0]=='x' || argv[0]=='X')   return modo;

            // Hora (hh:mm:ss)
            if (argv[2] == ':' && argv[5]==':'){
              vetor[2]=16*(argv[0]-0x30)+(argv[1]-0x30);  //Segundos
              vetor[1]=16*(argv[3]-0x30)+(argv[4]-0x30);  //Minutos
              vetor[0]=16*(argv[6]-0x30)+(argv[7]-0x30);  //Horas
              rtc_wr_vet(0,vetor,3);
            }

            //Data (dd/mm/aa)
            if (argv[2] == '/' && argv[5]=='/'){
              vetor[0]=16*(argv[0]-0x30)+(argv[1]-0x30);  //Dia
              vetor[1]=16*(argv[3]-0x30)+(argv[4]-0x30);  //Mês
              vetor[2]=16*(argv[6]-0x30)+(argv[7]-0x30);  //Ano
              rtc_wr_vet(4,vetor,3);
            }

        }

        rtc_rd_vet(0, vetor, 7);
        ser1_data_hora(vetor);
        ser1_crlf(1);
        lcdb_data(1,9,vetor);
        lcdb_hora(2,9,vetor);
        //ser1_hex8(x);
        //ser1_crlf(1);
        delay_10ms(100);
    }
    return modo;
}

//15 -WQ- Testar memo Flash
char modo_wq (char modo) {
    char qtd,argc[4], argv[15]; //Receber comandos
    char x,i,j;
    char vt[16];
    long wr_adr=0;  //Endereço para as escritas
    long rd_adr=0;  //Endereço para as leituras
    long er_adr=0;  //Endereço para apagar
    modo_ser1(modo);
    ser1_str(" Testar Flash\n");
    lcdb_str(1,1,"WQ");
    x=wq_sr1_rd();
    ser1_str("SR1 = ");
    ser1_hex8(x);

    x=wq_sr2_rd();
    ser1_str("  SR2 = ");
    ser1_hex8(x);
    ser1_crlf(1);

    w25_manuf_dev_id(vt);
    ser1_str("Manuf ID = ");
    ser1_hex8(vt[0]);
    ser1_spc(1);
    ser1_hex8(vt[1]);
    ser1_crlf(1);

    while(TRUE){
        qtd=0;
        while (qtd==0)  qtd=seri_cmdo(argc,argv,15);  //Esperar comando
        if (argv[0]=='x' || argv[0]=='X') return modo;

        //Leitura 256 bytes da Flash
        if (argv[0]=='r' || argv[0]=='R'){
            if (qtd>1){
                rd_adr=str_2_num(&argv[argc[1]]);
                rd_adr &= 0xFFFFFF0;        //Endereço múltiplo de 16
            }
            ser1_str("RD ");
            ser1_hex32(rd_adr);
            ser1_crlf(1);

            for (i=0; i<16; i++){
                wq_rd_blk(rd_adr, vt,16);
                ser1_hex32(rd_adr);
                ser1_char(':');
                ser1_spc(1);
                ser1_linha(vt);
                rd_adr+=16;
            }
            ser1_crlf(1);
        }

        // WR - Escrita de um padrão de 256 bytes, opera em blocos de 16 bytes
        if (argv[0]=='w' || argv[0]=='W'){
            if (qtd>1){
                wr_adr=str_2_num(&argv[argc[1]]);
                wr_adr &= 0xFFFFFF0;        //Endereço múltiplo de 16
            }
            ser1_str("WR ");
            ser1_hex32(wr_adr);
            ser1_crlf(1);
            x=0;
            for (i=0; i<16; i++){
                for (j=0; j<16; j++)
                    vt[j]=j+x;

                wq_wr_blk(wr_adr, vt,16);
                wr_adr+=16;
                x+=16;
            }
            ser1_str("OK");
            ser1_crlf(1);
        }

        // Erase - Apagar página de 4 KB (adr = aaaa aaaa   aaaa 0000   0000 0000)
        if (argv[0]=='e' || argv[0]=='E'){
            if (qtd>1){
                er_adr=str_2_num(&argv[argc[1]]);
                wr_adr &= 0xFFF000;        //Endereço múltiplo de 4 KB
            }
            ser1_str("Erase 4K: ");
            ser1_hex32(er_adr);
            wq_erase_4k(er_adr);
            ser1_str(" OK");
            ser1_crlf(1);
        }
    }
    return modo;
}

//16 -GPS- Testar GPS
// Usa TA0
char modo_gps  (char modo) {
    char x;
    gps_config();
    modo_ser1(modo);
    ser1_str(" Testar GPS\n");
    lcdb_str(1,1,"GPS");
    while(TRUE){
        if (seri_cmdo_x()==TRUE){
            TA0CTL=0;   //Parar o timer, desligar a serial do GPS
            return modo;
        }
        if (gps_tira(&x)==TRUE)
                ser1_char(x);
    }
    return modo;
}



char modo_0   (char modo) { modo_ser1(modo); return modo;}
char modo_1   (char modo) { modo_ser1(modo); return modo;}
char modo_2   (char modo) { modo_ser1(modo); return modo;}
char modo_3   (char modo) { modo_ser1(modo); return modo;}
char modo_4   (char modo) { modo_ser1(modo); return modo;}
char modo_5   (char modo) { modo_ser1(modo); return modo;}
char modo_6   (char modo) { modo_ser1(modo); return modo;}
char modo_7   (char modo) { modo_ser1(modo); return modo;}
char modo_8   (char modo) { modo_ser1(modo); return modo;}
char modo_9   (char modo) { modo_ser1(modo); return modo;}
char modo_16  (char modo) { modo_ser1(modo); return modo;}
char modo_17  (char modo) { modo_ser1(modo); return modo;}
char modo_18  (char modo) { modo_ser1(modo); return modo;}
char modo_19  (char modo) { modo_ser1(modo); return modo;}
char modo_20  (char modo) { modo_ser1(modo); return modo;}
char modo_99  (char modo) { ser1_str("Invalido");  return modo;}

// Imprimir modo no Serial
void modo_ser1(char modo){
    ser1_str("\r\nModo ");   ser1_dec8unz(modo);
}

