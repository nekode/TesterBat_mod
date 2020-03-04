/*
Оригинальное описание и схема здесь. http://srukami.inf.ua/testbat3.htm
А также фото и видео работы. https://youtu.be/BODFInBvmFw
Автор оригинального скетча srukami
МОД создан на модифицированом скетче под I2C LCD 1602 от arthemy https://www.youtube.com/user/arthemy https://mega.nz/#F!NApFHaaA!g-swQBanZrVbW4QhXdZBXQ

Изменено:
-обычный индикатор 1602 вместо индикатора с i2c модулем и авторского 5110,
-изменена схема - использованы транзисторы КТ814 и КТ315, добавлены входы АЦП для повышения точности (в авторском скетче не учитывается падение напряжения на открытом транзисторе),
-изменена нумерация входов-выходов (см. схему   ),
-реализовано грубое тестирование внутреннего сопротивления,
-добавлена индикация тока заряда,
-убраны дублирующие переменные (в авторском скетче была заморочка с преобразованием float > int),
-добавлен ИОН на TL431 для коррекции измерений при нестабильном напряжении питания,
-сделан выбор типа тестируемого аккумулятора (Li-Ion, Li-Pol, LiFePO4). По умолчанию стоит Li-Ion,
-сделан выбор тока разряда (малый, средний, большой), чтобы тестировать мощные аккумуляторы быстрее,
-добавлен бузер, сигнализирующий переключение между режимами и окончании цикла измерений,
-добавлено меню с настройками, чтобы менять параметры не перезагружая устройство
-добавлено меню настройки параметров,
-добавлен финальный заряд до напряжения хранения.
Было в планах (но реализацию оставляю желающим ковырять проект далее):
-сделать ещё один пункт "manual akk" в меню, где можно будет задать свои напряжения окончания разряда и заряда (с сохранением в EEPROM),
-оптимизировать алгоритм подсчёта времени разряда (на данный момент часы слегка отстают из-за фиксированого приращения по 5 секунд),
-сделать чтение с АЦП с оверсемплингом.


*/

#include <util/delay.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>
float R5 = 5.5;         //задаём сопротивление зарядного резистора R5
float R10 = 22.3;       //задаём сопротивление разрядного резистора R10
float R15 = 10.2;       //задаём сопротивление разрядного резистора R15
int adc_TL431 = 512;	//задаём напряжение на источнике опорного (в попугаях АЦП) 
byte EEPROM_check = 0; //переменная проверки содержимого ЕЕПРОМ
const int analogV = A0;       //назначаем аналоговый вход аккума
const int analogVC = A1;      //назначаем аналоговый вход заряда
const int analogVD = A2;      //назначаем аналоговый вход разряда
const int analogVD2 = A3;     //назначаем аналоговый вход разряда 2
const int analogV_TL431 = A4; //назначаем аналоговый вход коррекции напряжения
float coeff = 0.0;            //поправка на нестабильность напряжения
float volt = 0.0;             //задаем переменные
float R = 0.0;                //внутреннее сопротивление аккумулятора
float voltC = 0.0;            //Напряжение при заряде
float voltD = 0.0;            //Напряжение при разряде
float voltD2 = 0.0;           //Напряжение при разряде
float Vc = 0.0;               //Конечное напряжение заряда
float Vd = 0.0;               //Конечное напряжение разряда
float am = 0;                 //милиамперы
float amR = 0;                //милиамперы, ток определения внутреннего сопротивления
float amR2 = 0;               //милиамперы, ток определения внутреннего сопротивления
float amC = 0;                //милиамперы заряд
float capvrem= 0;             //временная емкость
float cap= 0;                 //емкость
float voltTEMP= 0;            //временная переменная для рассчёта внутреннего сопротивления
byte counteR = 0;             //счётчик циклов замера внутреннего сопротивления
byte knop = 0;                //переменная для хранения состояния кнопки
byte knop_menu = 1;           //переменная для хранения состояния кнопки
byte SEK = 0;                 //секунды
byte MIN = 0;                 //минуты
byte HOUR = 0;                //часы
byte akk_type = 0;            //переменная типа аккумулятора
byte discharge_type = 0;      //переменная типа разряда
byte recharge_control = 0;    //переменная для однократного подзаряда в конце
unsigned long prMillis = 0;
// initialize the library with the numbers of the interface pins       
LiquidCrystal lcd(12, 11, 5, 4, 19, 10);   // 19 - pinA5

void setup() 
{
  pinMode(analogV , INPUT);    //определяем тип порта (вход), АЦП
  pinMode(analogVC , INPUT);  //определяем тип порта (вход), АЦП
  pinMode(analogVD , INPUT);  //определяем тип порта (вход), АЦП
  pinMode(analogVD2 , INPUT);  //определяем тип порта (вход), АЦП
  pinMode(analogV_TL431 , INPUT);  //определяем тип порта (вход), АЦП
  pinMode(2, INPUT);         //определяем тип порта (вход), кнопка
  pinMode(3, INPUT);         //определяем тип порта (вход), кнопка
  pinMode(13, OUTPUT);   // бузер
  pinMode(9, OUTPUT);   // разряд
  pinMode(8, OUTPUT);   // заряд
  pinMode(7, OUTPUT);   //LED
  pinMode(6, OUTPUT);   //разряд2
  lcd.begin(16, 2); // размер и инициализация дисплея 
  lcd.clear();      // Чистим экран
  EEPROM.get(1, EEPROM_check); //читаем ячейку ЕЕПРОМ с адресом 1
  if (EEPROM_check == 55) //если находим в ней внесённое при настройке значение
	{
		EEPROM.get(2, R5);  	//считываем из ЕЕПРОМ сопротивление зарядного резистора R5
		EEPROM.get(6, R10);      	//считываем из ЕЕПРОМ  сопротивление разрядного резистора R10
		EEPROM.get(10, R15);       	//считываем из ЕЕПРОМ  сопротивление разрядного резистора R15
		EEPROM.get(14, adc_TL431);		//считываем из ЕЕПРОМ  напряжение на источнике опорного (в попугаях АЦП) 
	}
    if ((digitalRead (2) == HIGH) && (digitalRead (1)) == HIGH) // если при включении прибора обнаружили зажатыми обе кнопки
	{
	menu_config();
	}
	else
	{
		lcd.setCursor(0, 0); //курсор в начало первой строки
		lcd.print(" Tester Bat mod "); //заставка 
		_delay_ms(1000); //задержка заставки 
		lcd.clear();      // Чистим экран  
	}
  menu(); //вызов меню для начальной конфигурации
}

void loop()
{ 
  if (digitalRead(3)==HIGH)		//нажатие кнопки "меню"
    {
       knop_menu=1;
       digitalWrite(8,LOW);            //выключаем ключ зарядки аккумулятора
       digitalWrite(9,LOW);            //выключаем ключ нагрузки
       digitalWrite(6,LOW);            //выключаем ключ нагрузки 2
       digitalWrite(7,LOW);            //выключаем светодиод заряда
       _delay_ms(100);
       menu();
    }
coeff = adc_TL431/(float (analogRead(analogV_TL431)));      // считаем коэффициент погрешности опорного напряжения
volt = float (analogRead(analogV))*coeff*5/1024;      		// 5В - опорное напряжение
voltC = float (analogRead(analogVC))*coeff*5/1024;    		// Напряжение заряда
voltD = float (analogRead(analogVD))*coeff*5/1024;    		// Напряжение разряда
voltD2 = float (analogRead(analogVD2))*coeff*5/1024;  		// Напряжение разряда 2
  if (digitalRead(2)==HIGH)		//нажатие кнопки
  {
    tone(13, 1000, 70);
    knop++;
    lcd.clear();					      //чистим экран  
    if (knop>4){knop=1; }		  	// обнуляем четвёртое нажатие.  
    _delay_ms(250);
  }
  if (knop==0){    //сценарий устройство включено но незапущен тест
    lcd.setCursor(0, 0);
    lcd.print("Press 1Ch 2Rint ");
    lcd.setCursor(0, 1);
    lcd.print("3Dis 4FinCh ");
    lcd.print(volt, 1);  
    lcd.print("V           ");      
    _delay_ms(250);
  }
//-------------------------------------------------
  if (knop==1){		//отработка сценария зарядки после подключения
    lcd.clear();					      //чистим экран  
    cap= 0;			//обнуляем переменные
    am = 0;
    amR = 0;
    amR2 = 0;
    amC = 0;
    R = 0;
    SEK = 0; 
    MIN = 0; 
    HOUR = 0;
    counteR = 0;
    voltTEMP = 0;
    recharge_control = 0;
    digitalWrite(7,LOW);		//светодиод разряжено
    if (volt<Vc)
    {                   
      digitalWrite(8,HIGH);	//включаем ключ зарядки аккумулятора
      digitalWrite(9,LOW);		//отключаем ключ нагрузки
      amC=((voltC-volt)/R5)*1000;        //считаем ток в mA =Uтекущее/Rнагрузки,  *1000 получаем mA
      lcd.setCursor(0, 0);
      lcd.print("CHARGE: ");
      lcd.print(volt);     //выводим напряжение на аккумуляторе
      lcd.print("V        ");   
      lcd.setCursor(8, 1);
      lcd.print("I=");
      lcd.print(amC);        
    }
    if(volt>=Vc) {knop = 2;    tone(13, 1000, 150);}  // фиксируем зарядку АКБ, переходим в проверку внутреннего сопротивления
      _delay_ms(350);
  }
//----------------------------------------------------------------------------------------------------
  if (knop==2)            //замер внутреннего сопротивления
  {
   if (counteR<=252)
   {
       
        if (counteR<=230)      //первый цикл проверки
           {
            if (counteR==0) { lcd.clear(); } // Чистим экран
            digitalWrite(8,LOW);    //отключаем ключ зарядки аккумулятора
            digitalWrite(6,LOW);    //отключаем ключ нагрузки 2
            digitalWrite(9,HIGH);   //включаем ключ нагрузки
            amR=(voltD/R10);        //считаем ток
            lcd.setCursor(0, 0);
            lcd.print("Testing Rint ");
            lcd.setCursor(13, 0);   //ставим курсор в 14й столбец, первую строку (вспомогательные данные, можно закомментить строку)
            lcd.print("R10");        //используемый резистор (вспомогательные данные, можно закомментить строку)
            lcd.setCursor(0, 1);    //ставим курсор в первый столбец, вторую строку
            lcd.print(amR*1000,0);  //выводим значение тока разряда в мА
            lcd.print("mA ");       //
            lcd.print(volt,2);      //выводим напряжение на аккумляторе под нагрузкой
            lcd.print("V ");        //
            _delay_ms(500);         //задержка
            lcd.setCursor(0, 1);    //ставим курсор в первый столбец, вторую строку
            lcd.print("                ");        //чистим строку
            counteR++;              //инкрементируем счётчик
            if (counteR<=231)      //окончание первого цикла проверки
             {
               voltTEMP = volt;        //переносим значение напряжения под нагрузкой во временную переменную
             }
          }
       if (counteR>230 and counteR<=250)      //второй цикл проверки
          {
            if (counteR==231) { lcd.clear(); } // Чистим экран
            digitalWrite(8,LOW);    //отключаем ключ зарядки аккумулятора
            digitalWrite(9,LOW);    //отключаем ключ нагрузки           
            digitalWrite(6,HIGH);   //включаем ключ нагрузки 2
            amR2=(voltD2/R15);      //считаем ток
            lcd.setCursor(0, 0);
            lcd.print("Testing Rint");
            lcd.setCursor(13, 0);   //ставим курсор в 14й столбец, первую строку
            lcd.print("R15");        //используемый резистор (вспомогательные данные, можно закомментить строку)
            lcd.setCursor(0, 1);    //ставим курсор в первый столбец, вторую строку
            lcd.print(amR2*1000,0); //выводим значение тока разряда в мА
            lcd.print("mA ");       //
            lcd.print(volt,2);      //выводим напряжение на аккумляторе под нагрузкой
            lcd.print("V ");        //
            counteR++;              //инкрементируем счётчик
            _delay_ms(500);         //задержка         
            lcd.setCursor(0, 1);    //ставим курсор в первый столбец, вторую строку
            lcd.print("                ");        //чистим строку
          }

        if (counteR==251)             //заканчиваем проверку сопротивления
           {
             R = ((voltTEMP-volt)/(amR2-amR));  //считаем внутреннее сопротивление аккумулятора
             digitalWrite(9,LOW);         //выключаем ключ нагрузки
             digitalWrite(6,LOW);         //выключаем ключ нагрузки 2
             digitalWrite(8,LOW);         //выключаем ключ зарядки аккумулятора
             counteR++;                   //инкрементируем счётчик
             lcd.clear();  // Чистим экран
             lcd.setCursor(0, 1);         //ставим курсор в первый столбец, вторую строку
             lcd.print("Rin=");           //
             lcd.print(R,3);              //выводим значение внутреннего сопротивления
           }
        if (counteR>=252)  //восстанавливаем заряд аккумулятора перед тестом ёмкости, переходим в разряд
            {
             digitalWrite(9,LOW);     //выключаем ключ нагрузки
             digitalWrite(6,LOW);     //выключаем ключ нагрузки 2
             digitalWrite(8,HIGH);    //включаем ключ зарядки аккумулятор
             lcd.setCursor(0, 0);         //ставим курсор в первый столбец, первую строку
             lcd.print("re-charging");        //дозарядка до полного
             lcd.setCursor(11, 1);         //ставим курсор в 12й столбец, вторую строку
             lcd.print(volt);        //вывод текущего напряжения на аккумуляторе
             lcd.print("V");         //
             _delay_ms(350);
             lcd.setCursor(15, 0);         //ставим курсор в 16й столбец, первую строку
             lcd.print("*");               //мигающий значок заряда
             _delay_ms(350);
             lcd.setCursor(15, 0);         //ставим курсор в 16й столбец, первую строку
             lcd.print(" ");               //мигающий значок заряда
             lcd.setCursor(11, 1);         //ставим курсор в 12й столбец, вторую строку
             lcd.print("     ");         //
             
           if (volt>=Vc)             //если зарядился - переходим в разряд
                {
                 digitalWrite(8,LOW);    //выключаем ключ зарядки аккумулятора
                 knop = 3;                    //фиксация окончания замера внутреннего сопротивления, переход к замеру ёмкости
                 tone(13, 1000, 150);   //тональный сигнал смены цикла
                }
            } 
   }
  }
//----------------------------------------------------------------------------------------------------------------
  if (knop==3){					//проверка сценария для разрядки
    lcd.clear();      // Чистим экран
    if (volt>Vd)
    {
          digitalWrite(8,LOW);		//отключаем ключ зарядки аккумулятора
             if (discharge_type==1)
                {
                 digitalWrite(9,HIGH);  //включаем ключ нагрузки 
                }
            if (discharge_type==2)
                {
                 digitalWrite(6,HIGH);  //включаем ключ нагрузки 
                }
            if (discharge_type==3)
                {
                 digitalWrite(6,HIGH);  //включаем ключ нагрузки 
                 digitalWrite(9,HIGH);  //включаем ключ нагрузки 
                }
      //#########цикл отсчета
      if (millis()-prMillis>=5000){
        prMillis=millis();
        SEK=SEK+5;
            if (discharge_type==1)
                {
                 am=(voltD/R10)*1000;        //считаем ток в mA =Uтекущее/Rнагрузки,  *1000 получаем mA
                }
            if (discharge_type==2)
                {
                 am=(voltD2/R15)*1000;        //считаем ток в mA =Uтекущее/Rнагрузки,  *1000 получаем mA
                }
            if (discharge_type==3)
                {
                am=((voltD/R10)*1000)+((voltD2/R15)*1000);        //считаем ток в mA =Uтекущее/Rнагрузки,  *1000 получаем mA
                }
        capvrem=am/720; cap=cap+capvrem;	//считаем емкость в mAh, при замере раз в 5 сек (в часе 3600сек / 5 = 720)
        if (SEK>59){SEK=0;MIN++;}
        if (MIN>59){MIN=0;HOUR++;}
        if (HOUR>23) HOUR=0;
      }
        //######################## 
      lcd.setCursor(0, 0);
      lcd.print("DisCh  ");
      lcd.print("I=");		//вывод на экран
      lcd.print(am, 0);		//вывод на экран
      lcd.print("mA ");		//вывод на экран
      lcd.setCursor(0, 1);
      lcd.print(volt); 
      lcd.print("V  Q=");	//вывод на экран
      lcd.print(cap, 0);		//вывод на экран
      lcd.print("mAh    ");	//вывод на экран
      lcd.setCursor(12, 1);
    }
    _delay_ms(500);
    if (volt<=Vd){knop=4; tone(13, 1000, 150);}  //фиксация окончания разрядки и замера емкости   
  }
//-------------------------------------------------
  if (knop==4){					              //сценарий окончания подсчета емкости, вывод на экран и зарядка.
    lcd.setCursor(10, 1);
    lcd.print(volt, 1);               //вывод на экран текущего напряжения аккумулятора
    lcd.print("V");

   switch (recharge_control)
    {
      case 0:                          //тест окончен, но аккумулятор полностью разряжен, выводим результаты на экран
       digitalWrite(8,LOW);            //выключаем ключ зарядки аккумулятора
       digitalWrite(9,LOW);            //выключаем ключ нагрузки
       digitalWrite(6,LOW);            //выключаем ключ нагрузки 2
       digitalWrite(7,LOW);            //выключаем светодиод заряда
       lcd.setCursor(0, 0);            //выводим значение внутреннего сопротивления
       lcd.print("R=");
       lcd.print(R,3);
       lcd.print(" ");
       lcd.setCursor(8, 0);
      if (HOUR<1){ lcd.print("00");}   //вывод времени разряда на экран
         else {lcd.print(HOUR);}
       lcd.print(":");
      if (MIN<1){ lcd.print("00");}    //вывод на экран
         else {lcd.print(MIN);}
       lcd.print(":");
       lcd.print(SEK);
       lcd.setCursor(0, 1);
//     lcd.print("Q=");                //вывод на экран
       lcd.print(cap, 1);              //вывод на экран
       lcd.print("mAh ");              //вывод на экран
       recharge_control=1;             //инкремент счётчика
       _delay_ms(5);
       break;                          //выход из case

      case 1:                      //тест окончен, результаты отображены, заряжаем
//            if (volt<Vc)               //после того как посчитали емкость заряжаем
            if (volt<(Vc - 0.3))               //после того как посчитали емкость заряжаем (до напряжения хранения)
             {
              digitalWrite(8,HIGH);    //включаем ключ зарядки аккумулятора
              lcd.setCursor(15, 1);    //столбец 16, строка 2
              lcd.print(" ");          //мигаем звёздочкой индикатора заряда
              _delay_ms(250);
              lcd.setCursor(15, 1);    //столбец 16, строка 2
              lcd.print("*");         //мигаем звёздочкой индикатора заряда
              _delay_ms(250);
              }
//            if (volt>=Vc)   //после того как зарядили переходим к поддержанию заданного напряжения
            if (volt>=(Vc - 0.3))   //после того как зарядили переходим к поддержанию заданного напряжения
            {
              digitalWrite(8,LOW);             //отключаем ключ зарядки аккумулятора
              digitalWrite(7,HIGH);            //включаем светодиод заряда
              recharge_control=2;              //инкрементируем счётчик
              tone(13, 500, 300);
              _delay_ms(301);
              tone(13, 700, 300);
              _delay_ms(301);
              tone(13, 900, 300);
              _delay_ms(301);
            }
              break;

      case 2:
//    if (volt<(Vc-0.2))                 //после заряда поддерживаем напряжение на 0.2 вольта ниже 
    if (volt<(Vc-0.5))                 //после заряда поддерживаем напряжение
           {
              digitalWrite(8,HIGH);    //включаем ключ зарядки аккумулятора
              _delay_ms(150);
           }
		   
//    if (volt>=(Vc-0.2))                //после заряда поддерживаем напряжение на 0.2 вольта ниже
    if (volt>=(Vc-0.5))                //после заряда поддерживаем напряжение	
        {
         digitalWrite(8,LOW);          //отключаем ключ зарядки аккумулятора
         _delay_ms(500);
        }
            break;

    }
  }
}


void menu()                    //меню
{
knop=0;                        //обнуляем используемые переменные
akk_type=1;                    //по умолчанию Li-Ion
discharge_type=1;              //по умолчанию минимальный ток разряда
_delay_ms(150);              //задержка - антидребезг
while  (knop_menu!=0)            //пока не произведены начальные настройки остаёмся в меню
{
    if (digitalRead(2)==HIGH)    //нажатие кнопки
  {
    lcd.clear();                //чистим экран 
    knop++;                     //инкремент кнопки
    tone(13, 1000, 70);         //бипер
    if (knop>3){knop=1; }       // обнуляем нажатие  
    _delay_ms(75);              //задержка - антидребезг
  }
    if (digitalRead(3)==HIGH)    //нажатие кнопки
  {
    knop=0; 
    lcd.clear();                //чистим экран 
    knop_menu++;                //инкремент кнопки
    tone(13, 1000, 70);         //бипер
    if (knop_menu>3){knop_menu=1; }       // обнуляем нажатие  
    _delay_ms(75);              //задержка - антидребезг
  }     
switch(knop_menu)                //выбор типа аккумулятора и тока разряда
 {
case 1:                //выбор типа аккумулятора
     {
              lcd.setCursor(0, 0);                  //индикация текущего типа аккумулятора
              lcd.print("  akkum. type:  ");
                     if (knop==0)                    
                       {
                         lcd.setCursor(0, 0);
                         lcd.print("change akk type?");

                       if (akk_type==1)
                         {
                           lcd.setCursor(0, 1);
                           lcd.print("Li-Ion 2.9V-4.1V");
                         }
                       if (akk_type==2)
                         {
                           lcd.setCursor(0, 1);
                           lcd.print("Li-Pol 2.9V-4.2V");
                         }
                       if (akk_type==3)
                         {
                           lcd.setCursor(0, 1);
                           lcd.print("Li-Fe 2.8V-3.6V ");
                         }

                      }

                if (knop==1)                    //выбор литий-ионной батареи
                {
                  lcd.setCursor(0, 1);
                  lcd.print("Li-Ion 2.9V-4.1V");
                  akk_type=1;
                }
                if (knop==2)                    //выбор литий-полимерной батареи
                {
                  lcd.setCursor(0, 1);
                  lcd.print("Li-Pol 2.9V-4.2V");
                  akk_type=2;
                }
                if (knop==3)                    //выбор литий-железо-фосфатной батареи
                {
                  lcd.setCursor(0, 1);
                  lcd.print("Li-Fe 2.8V-3.6V ");
                  akk_type=3;
                }
            _delay_ms(250);
            break;
     }


case 2:                       //выбор тока разряда
    {
              lcd.setCursor(0, 0);
              lcd.print("discharge type: ");


             if (knop==0)                
              {
                  lcd.setCursor(0, 0);           //индикация текущей уставки тока разряда
                  lcd.print("change I disch.?");

                 if (discharge_type==1)       //минимальный ток
                   {
                     lcd.setCursor(0, 1);
                     lcd.print("minimum current ");
                   }

                 if (discharge_type==2)      //средний ток
                   {
                     lcd.setCursor(0, 1);
                     lcd.print("middle current  ");
                   }

                 if (discharge_type==3)      //максимальный ток
                   {
                     lcd.setCursor(0, 1);
                     lcd.print("maximum current ");
                   }
              }

                if (knop==1)                //выбор минимального тока разряда (R10)
                {
                  lcd.setCursor(0, 1);
                  lcd.print("minimal current ");
                  discharge_type=1;
                }
                if (knop==2)               //выбор среднего тока разряда (R15)
                {
                  lcd.setCursor(0, 1);
                  lcd.print("middle current  ");
                  discharge_type=2;
                }
                if (knop==3)               //выбор максимального тока разряда (R10+R15)
                {
                  lcd.setCursor(0, 1);
                  lcd.print("maximum current ");
                  discharge_type=3;
                }
           _delay_ms(250);                //задержка
           break;
    }

case 3:                                       //сохранение параметров и переход к тесту
    {
              lcd.setCursor(0, 0);
              lcd.print("  save and go   ");
              lcd.setCursor(0, 1);
              lcd.print("    to test?    ");
                if(knop==1)
                  {
                   knop_menu=0;

                     if (akk_type==1)
                         {
                           Vd=2.9;
                           Vc=4.1;
                         }
                     if (akk_type==2)
                         {
                           Vd=2.9;
                           Vc=4.2;
                         }
                     if (akk_type==3)
                         {
                           Vd=2.8;
                           Vc=3.6;
                         }
                  }
           _delay_ms(250);                //задержка
            break;
    }
 }
}
       knop=0;                             //обнуляем переменные
       knop_menu=0;                        //обнуляем переменные
}

void menu_config()
	{
		uint32_t millis_timeout = millis();
		tone(13, 900, 300);
		lcd.print("  config  mode  ");
		_delay_ms(1500);
		lcd.clear();      // Чистим экран
		byte menu_step = 1;
		while  (menu_step!=0)            //пока не произведены начальные настройки остаёмся в меню
		{
		lcd.clear();      // Чистим экран
		if ((millis() - millis_timeout) > 7000)
			{
				menu_step++;
				millis_timeout = millis();
				tone(13, 900, 300);
				_delay_ms(301);
			}
		switch(menu_step)                //выбор настраиваемого параметра
		 {
		case 1:                //выбор сопротивления R5
			 {
					lcd.setCursor(0, 0);
					lcd.print("R5 = ");
					lcd.setCursor(0, 1);
					lcd.print(R5);
					if (digitalRead(3)==HIGH)    //нажатие кнопки
						{
						R5 = R5 + 0.1;
						millis_timeout = millis();
						}
					else if (digitalRead(2)==HIGH)    //нажатие кнопки
						{
						R5 = R5 - 0.1;
						millis_timeout = millis();
						}
					_delay_ms(250);
					break;
			 }


		case 2:                       //выбор сопротивления R10
			{
					lcd.setCursor(0, 0);
					lcd.print("R10 = ");
					lcd.setCursor(0, 1);
					lcd.print(R10);
					if (digitalRead(3)==HIGH)    //нажатие кнопки
						{
						R10 = R10 + 0.1;
						millis_timeout = millis();
						}
					else if (digitalRead(2)==HIGH)    //нажатие кнопки
						{
						R10 = R10 - 0.1;
						millis_timeout = millis();
						}			
				   _delay_ms(250);                //задержка
				   break;
			}

		case 3:                                       //выбор сопротивления R15
			{
					lcd.setCursor(0, 0);
					lcd.print("R15 = ");
					lcd.setCursor(0, 1);
					lcd.print(R15);
					if (digitalRead(3)==HIGH)    //нажатие кнопки
						{
						R15 = R15 + 0.1;
						millis_timeout = millis();
						}
					else if (digitalRead(2)==HIGH)    //нажатие кнопки
						{
						R15 = R15 - 0.1;
						millis_timeout = millis();
						}				
				   _delay_ms(250);                //задержка
					break;
			}
		case 4:                                       //подстройка опорного напряжения
			{
					if (digitalRead(3)==HIGH)    //нажатие кнопки
						{
							adc_TL431++;
							millis_timeout = millis();
						}
					else if (digitalRead(2)==HIGH)    //нажатие кнопки
						{
							adc_TL431--;
							millis_timeout = millis();
						}
					coeff = adc_TL431/(float (analogRead(analogV_TL431)));      // считаем коэффициент погрешности опорного напряжения
					volt = float (analogRead(analogV_TL431))*coeff*10/1024;      		// 5В - опорное напряжение
					lcd.setCursor(0, 0);
					lcd.print("Vcc = ");
					lcd.print(volt);
					lcd.print("V");
					lcd.setCursor(0, 1);
					lcd.print("adc_TL431 = ");
					lcd.print(adc_TL431);
				   _delay_ms(250);                //задержка
					break;
			}
		case 5:                                       //сохранение переменных
			{
				lcd.setCursor(0, 0);
				lcd.print("    saving...   ");
				uint16_t EEPROM_counter = 0;	
				EEPROM.get(1, EEPROM_check); //читаем ячейку ЕЕПРОМ с адресом 1
				if (EEPROM_check == 55) //если находим в ней внесённое при настройке значение
					{
						EEPROM.get(16, EEPROM_counter); //читаем ячейки 16-17
						EEPROM_counter++; //инкремент прочитанного значения
					}
				_delay_ms(250);                //задержка
				EEPROM_check = 55;
				lcd.setCursor(7, 1);
				lcd.print(EEPROM_counter);
				EEPROM.put(1, EEPROM_check);  	//записываем в ЕЕПРОМ сопротивление зарядного резистора R5
				EEPROM.put(16, EEPROM_counter);  	//записываем в ЕЕПРОМ счётчик циклов перезаписи		
				EEPROM.put(2, R5);  	//записываем в ЕЕПРОМ сопротивление зарядного резистора R5
				EEPROM.put(6, R10);      	//записываем в ЕЕПРОМ  сопротивление разрядного резистора R10
				EEPROM.put(10, R15);       	//записываем в ЕЕПРОМ  сопротивление разрядного резистора R15
				EEPROM.put(14, adc_TL431);		//записываем в ЕЕПРОМ  напряжение на источнике опорного (в попугаях АЦП) 
				_delay_ms(250);                //задержка
				menu_step = 0;
				break;
			}
		default :
			{
				menu_step = 0;
				break;			
			}
		 }
		}
		lcd.setCursor(0, 0);
		lcd.print("      saved      ");
		_delay_ms(1000);                //задержка
		return;
	}
