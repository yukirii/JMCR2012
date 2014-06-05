// データ転送用ヘッダ
// 初期設定　init_sci1(SMR,BRR);
// 文字列転送　sci1_strtx("文字列");
// 数字転送　sci_put_dig(数値,桁数);

void init_sci( char , char );
unsigned char in_sci( void );
void sci_tx( char );
void sci_strtx(char *);
void sci_put_dig( long , char );
void sci_change_num( char );
void sci_put_num( long , char );

// 初期設定
void init_sci( char smr, char brr )
{
    int i;
    
	SCI1.SCR.BYTE = 0x00;	// 停止
	SCI1.SMR.BYTE = smr;		// SMRの値
	SCI1.BRR = brr;		// BRRの値
	for(i=0;i<10000;i++);	// ウェイト
	SCI1.SCR.BYTE = 0x30;	// Tx,Rx許可,割り込みなし
	SCI1.SSR.BYTE &= 0x80;	// エラーフラグ
}

// データ受信
unsigned char in_sci(void)
{
	unsigned char	data;

	while( !(SCI1.SSR.BYTE & 0x78 ));		// 受信待ち
	switch( SCI1.SSR.BIT.RDRF ){		// レシーブフラグチェック
		case 1:
			data = SCI1.RDR;		// 受信データ代入
			SCI1.SSR.BIT.RDRF = 0;	// フラグクリア
			return( data );
   			break;
		default:				// レシーブエラー
			SCI1.SSR.BYTE &= 0x87;	// エラーフラグクリア
			return( 0xff );		// エラー時　0Xff　戻り値
	}
}

// データ転送(1byte)
void sci_tx(char data)
{
	while(SCI1.SSR.BIT.TDRE == 0);	// 空き待ち
	SCI1.TDR = data;				// データセット
	SCI1.SSR.BIT.TDRE = 0;			// フラグクリア
}

// 文字列転送
void sci_strtx(char *str)
{
	while(*str != '\0'){
		sci_tx(*str);
		str++;
	}
}
void sci_put_num(long num, char keta)
{
    char temp_chr[11];
    int  i,a,x,y,kari;

	if(keta>10)	return; // Error over 10

    i = 0;  a = 0;  x = 1; y = 0; kari = num;

    if(kari == 0) a++;
    else{
    	while( kari != 0 ){
    		kari /= 10;
    		a++;
    		}
    	}
    
    for(i = 1 ;i < a ; i++) x *= 10;
    
    for(i=1 ; i<=keta ; i++){
    	if(i <= keta-a) temp_chr[i-1] = 0x20;
    	else{
    		if( num < 0 ){
    			num=-(num);
    			temp_chr[i-2]='-';
    		}
    		y=num/x;
    		temp_chr[i-1]=(0x30)|(char)y;
    		num=num-(y*x);
    		x/=10;
    	}
    }
    	
    temp_chr[i-1]='\0';
    
    sci_strtx(temp_chr);
    
    //i=0;
    //while(temp_chr[i]!='\0'){
    //	sci_change_num(  temp_chr[i] );
    //	i++;
    //}	
   
}
// 数値転送,マイナス未対応,要改造
void sci_put_dig(long num, char keta)
{
    char temp_chr[11];
    int  i,a,x,y,kari;

	if(keta>10)	return; // Error over 10

    i = 0;  a = 0;  x = 1; y = 0; kari = num;

    if(kari == 0) a++;
    else{
    	while( kari != 0 ){
    		kari /= 10;
    		a++;
    		}
    	}
    
    for(i = 1 ;i < a ; i++) x *= 10;
    
    for(i=1 ; i<=keta ; i++){
    	if(i <= keta-a) temp_chr[i-1] = 0x20;
    	else{
    		if( num < 0 ){
    			num=-(num);
    			temp_chr[i-2]='-';
    		}
    		y=num/x;
    		temp_chr[i-1]=(0x30)|(char)y;
    		num=num-(y*x);
    		x/=10;
    	}
    }
    	
    temp_chr[i-1]='\0';
    i=0;
    while(temp_chr[i]!='\0'){
    sci_change_num(  temp_chr[i] );
    i++;
    }	
   
}
// 数値→文字+転送
void sci_change_num(char num_1byte)
{
    num_1byte |= 0x30;		//アスキー変換
    sci_tx( num_1byte );
}
