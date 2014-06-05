//	LCD用ヘッダファイル
//	Vertex_lcd_05.h

//  プリプロセス
#define		LCD_D4	P4.DR.BIT.B0
#define		LCD_D5	P4.DR.BIT.B1
#define		LCD_D6	P4.DR.BIT.B2
#define		LCD_D7	P4.DR.BIT.B3
#define		LCD_RS	P4.DR.BIT.B4
#define		LCD_E	P4.DR.BIT.B6

//  プロトタイプ宣言
void	init_lcd(void);
void	lcd_write(int , char *);
void	lcd_write_cl(int, char *);
void	lcd_print(char *);
void	lcd_clear(int );
void	lcd_home(int );
void	lcd_cursor(int , int );
void	lcd_out8(unsigned char );
void	lcd_out4(int , unsigned char );
void	lcd_put_dig(long , int );
void	lcd_put_num(long , int );
void	lcd_put_flo(float , float);
void	lcd_change_num(int );
void	lcd_out_time(unsigned long );

//  初期化
void init_lcd(void)
{                                             
	unsigned int i;        
	
	lcd_out8(0x03);		//  Function Set 1
    lcd_out8(0x03);		//  Function Set 2
    lcd_out8(0x03);		//  Function Set 3
	lcd_out8(0x02);		//  Function Set
    //  初期設定
	lcd_out4(0, 0x28);	//  LCD Function Set
				//  Mode 4Bit / Line 2 / Font 5*7dots
	lcd_out4(0, 0x0C);	//  Display On/Off Control
				//  Display On / Cursor On / Blink On
	lcd_out4(0, 0x06);	//  Entry Mode Set
				//  Cursor Increment / Display_shift OFF
	lcd_clear(0);		//  LCD All Clear 
	
}

//  LCD書き込み(行頭から)
void lcd_write(int writeline, char *buff)
{
	switch(writeline) {
		case 1:				//  Line 1
			lcd_home(1);
			lcd_print(buff);
			break;
		case 2:				//  Line 2
			lcd_home(2);
			lcd_print(buff);
			break;
		default:
			break;
	}
}
void lcd_write_cl(int writeline, char *buff)
{
	switch(writeline) {
		case 1:				//  Line 1
			lcd_clear(1);
			lcd_print(buff);
			break;
		case 2:				//  Line 2
			lcd_clear(2);
			lcd_print(buff);
			break;
		default:
			break;
	}
}
//  LCD書き込み(カーソル位置から)
void lcd_print(char *buff)
{
	int count=0;

	while(*buff != '\0') {
		lcd_out4(1, *buff);
		buff++;
		count++;
		if (count==16) {
			break;
		}
	}
}

//  ビットデータの送出(モード指定)
void lcd_out8(unsigned char dat)
{
	int i;
          
	LCD_RS = 0;
	LCD_E = 1;
	LCD_D4 = (dat & 0x01);
	dat = dat >> 1;
	LCD_D5 = (dat & 0x01);
	dat = dat >> 1;
	LCD_D6 = (dat & 0x01);
	dat = dat >> 1;
	LCD_D7 = (dat & 0x01);

	for (i=0; i<150; i++);		//  wait 83us

	LCD_E = 0;

	for (i=0; i<150; i++);		//  wait 83us

	for (i=0;i<6500;i++);		//  wait 4ms
}

//  ビットデータの送出
void lcd_out4(int rs, unsigned char dat)
{
	unsigned char bk;
	int i;
	
	if(dat=='\0')dat=' ';
	bk=dat;
	
	LCD_RS = rs;
	LCD_E = 1;
	
	// 上位４ビット送出
	bk = bk >> 4;
	LCD_D4 = (bk & 0x01);
	bk = bk >> 1;
	LCD_D5 = (bk & 0x01);
	bk = bk >> 1;
	LCD_D6 = (bk & 0x01);
	bk = bk >> 1;
	LCD_D7 = (bk & 0x01);

	for (i=0; i<150; i++);		//  wait 83us
	LCD_E = 0;
	for (i=0; i<150; i++);		//  wait 83us
	LCD_E = 1;

	// 下位４ビットを送出 
	LCD_D4 = (dat & 0x01);
	dat = dat >> 1;
	LCD_D5 = (dat & 0x01);
	dat = dat >> 1;
	LCD_D6 = (dat & 0x01);
	dat = dat >> 1;
	LCD_D7 = (dat & 0x01);
	
	for (i=0; i<150; i++);		//  wait 83us
	LCD_E = 0;
	for (i=0; i<150; i++);		//  wait 83us
	
	if(rs == 0)for(i=0;i<6500;i++);	//  wait 4ms

	LCD_RS=0;
}

//  LCD表示のクリア
void lcd_clear(int clearline)
{
	switch(clearline) {
		case 0:				//  All Clear
			lcd_out4(0,0x01);
			break;
		case 1:				//  Line 1 Clear
			lcd_home(1);
			lcd_print("                ");
			lcd_home(1);
			break;
		case 2:				//  Line 2 Clear
			lcd_home(2);
			lcd_print("                ");
			lcd_home(2);
			break;
		default:
			break;
	}
}

//  カーソルの移動(行頭)
void lcd_home(int homeline)
{
	switch(homeline) {
		case 1:				//  Line 1
			lcd_out4(0,0x02);
			break;
		case 2:				//  Line 2
			lcd_out4(0,0xc0);
			break;
		default:
			break;
	}
}

//  カーソルの移動(任意指定)
void lcd_cursor(int cursorline, int x)
{
	if ((cursorline == 1 || cursorline == 2) && (x > 0 && x < 17)) {
		cursorline--;
		x--;
		lcd_out4(0,0x80+0x40*cursorline+x);
	}
}

//  数値データの表示(整数H)
void lcd_put_dig(long cnt, int keta)
{
    char temp_chr;
    int  i,flag=0;
    long temp_div=1;

    if(cnt==(-1))lcd_print(" -1");
   
    for(i=1; i<keta; i++)temp_div *= 10;
    for(i=0; i < keta; i++) {
        temp_chr = (cnt / temp_div) % 10;
        if(temp_chr!=0)flag=1;
        temp_div /= 10;
        if(temp_div==0&&!(cnt>0))lcd_out4(1,'0');
        else if(!flag)lcd_out4(1,' ');
        else lcd_change_num(temp_chr);
    }
}
//  数値データの表示(整数L)
void lcd_put_num(long cnt, int keta)
{
    char temp_chr;
    int  i,flag=0;
    long temp_div=1;

    for(i=1; i<keta; i++)temp_div *= 10;
    for(i=0; i < keta; i++) {
        temp_chr = (cnt / temp_div) % 10;
        temp_div /= 10;
        lcd_change_num(temp_chr);
    }
}
//  数値データの表示(実数)
void lcd_put_flo(float cnt, float keta)
{
    char	temp_chr;
    int		i,flag=0;
    long	temp_div=1,
    		dig,ketad,flo,ketaf;
    
    /*
    if(cnt<0){
    	lcd_out4(1,'-');
    	cnt*=-1;
    }
    else lcd_out4(1,' ');
    */
    dig=flo=ketad=ketaf=0;
    dig=cnt;
    ketad=keta;
    
    lcd_put_dig(dig,ketad);
    lcd_out4(1,'.');
    
    ketaf=keta*10;
    ketaf-=(ketad*10);
    temp_div = 10;
    for(i=1; i<ketaf; i++)temp_div *= 10;
    dig*=temp_div;
    flo=cnt*temp_div;
    flo-=dig;
    
    lcd_put_num(flo,ketaf);
}
//  数値データ(1文字,変換)
void lcd_change_num(int cnt)
{
    cnt += '0';
    lcd_out4(1,cnt);
}
//  時間表示
void lcd_out_time(unsigned long time)
{
	long min,sec,msec;
	min=sec=msec=0;
	time/=10;         
	
	while(time>=6000){
		time-=6000;
		min++;
	}
	sec=time/100;
	msec=time-(sec*100);
	if(min!=0){
		lcd_change_num(min);
		lcd_out4(1,':');
	}
	else lcd_print("  ");
	if(min!=0)lcd_put_dig(sec,2);
	else lcd_put_num(sec,2);
	lcd_out4(1,':');
	lcd_put_num(msec,2);
}