/**
 * Projeto: ESP READER
 * Descrição: Implementação de um leitor de tags RFID
 * com comunicação Serial e TCP/IP 
 * (Tentativa de implementação de um mercado automático para disciplina de engenharia de software) 
 */


#include<EEPROM.h>
#include<ESP8266WiFi.h>
#include <SPI.h>
#include <MFRC522.h>

/* Pinos utilizados:
   RST   -> D3 -> GPIO0
   NSS   -> D8 -> GPIO15
   MOSI  -> D7 -> GPIO13
   MISO  -> D6 -> GPIO12
   SCK   -> D5 -> GPIO14
   RED   -> D4 -> GPIO2
   GREEN -> D2 -> GPIO4
   BLUE  -> D1 -> GPIO5
*/

#define RED_PIN 2
#define GREEN_PIN 4
#define BLUE_PIN 5

#define RST_PIN 0
#define SS_PIN 15

#define TIMEOUT 20000
#define SSID_LENGTH 11
#define PASSWORD_LENGTH 11
#define TAG_LENGTH 16

//Constantes para decodificação de mensagens
  enum CLIENT_STS
    {
        ERR        = 0b0000,
        READY      = 0b0001,
        PROCESSING = 0b0010,
        CON_CONFIG = 0b0011,
        ASLEEP     = 0b0100,
        UNKOWN_STS = 0b0101,
    };

    enum SERVER_MSG
    {
        GET_STATUS = 0b00010000,
        SET_STATUS = 0b00100000,
        GET_MODE   = 0b00110000,
        SET_MODE   = 0b01000000,
        RESET_MEM  = 0b01010000,
        TURNOFF    = 0b01100000,
        EDIT       = 0b01110000,
        CONTINUE   = 0b10000000,
        BREAK      = 0b10010000
    };

    enum CLIENT_MODE
    {
        READ      = 0b0000,
        WRITE     = 0b0001,
        UNKOWN_MD = 0b0010
    };

    enum CLIENT_MSG
    {
        CURRENT_STATUS = 0b00010000,
        CURRENT_MODE   = 0b00100000,
        NEW            = 0b00110000,
        END            = 0b01000000,
        OP_SUCCESS     = 0b01010000
    };
    
//struct utilizada para configurar os parâmetros de conexão
typedef struct {
  unsigned char ssid[SSID_LENGTH];
  unsigned char password[PASSWORD_LENGTH];
  byte ip[4];
  int port;
} net_config;

//classe utilizada para comunicar esp_client às interfaces externas
/*ERROS:
  1. Falha na conexão WiFi
  2. Falha na conexão com o Servidor
  3. Serial não disponível
*/
class esp_client: public WiFiClient
{
  private:
    byte m_status;
    byte m_mode;
    bool m_is_configured;
    int m_error_code;
    net_config m_config;

  protected:
    void set_error(int code);

  public:
    esp_client();
    int wifi_connect();
    int server_connect();
    int serial_connect();

    void turn_off();
    void reset_mem();

    byte get_status();
    byte get_mode();
    int get_msg(byte* msg, int size);
    int get_msg(int* msg);
    int get_msg(byte* msg);
    int get_config(net_config* cfg);
    int get_error_code();

    int send_msg(byte* msg, int size);
    int send_msg(byte msg);

    bool is_configured();
    void set_mode(byte mode);
    void set_status(byte status);
    int set_config(net_config* nc);

    void print_config();
    void light_pattern(byte status);

    void wait_serial();
    void send_update();
    
};


/*ERROS:
  1. Falha na conexão WiFi
  2. Falha na conexão com o Servidor
  3. Serial não disponível
*/
class esp_reader: public esp_client, public MFRC522
{
  private:
    //Chave de acesso para leitura e escrita.
    MFRC522::MIFARE_Key m_key;

  public:
    esp_reader(int ss_pin, int rst_pin);
    int get_tag_data(byte buffer[TAG_LENGTH]);
    int set_tag_data(byte buffer[TAG_LENGTH]);
};

esp_reader esp(SS_PIN,RST_PIN);
char mem[5] = {0};

void setup() {

  Serial.begin(115200, SERIAL_8N1);

  pinMode(BLUE_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(RED_PIN, OUTPUT);
  Serial.println("Lendo:");
  Serial.println(mem);

  if (esp.is_configured())
  {
    if (esp.wifi_connect() != -1) {
      if (esp.server_connect() != -1)
      {
        esp.set_status(READY);
        esp.light_pattern(READY);
      }else
      {
        esp.serial_connect();
      }
    }else
    {
      esp.serial_connect(); 
    }
  }else
  {
    esp.serial_connect(); 
  }
}

void loop()
{
  byte status = esp.get_status();
  byte cmd = 0;
  byte mode = esp.get_mode();
  esp.get_msg(&cmd);

  if (status == READY)
  {
    switch (cmd&0xf0)
    {
      case CONTINUE:
        if (mode == READ)
        {
          process_itens();
        } else if (mode == WRITE)
        {
          register_itens();
        }
        break;

      case SET_MODE:
        if((cmd&0b00001111) == READ)
          esp.set_mode(READ);
        else if((cmd&0b00001111) == WRITE)
        {
          esp.set_mode(WRITE);
        }
        break;

      case GET_MODE:
        esp.send_msg(CURRENT_MODE|esp.get_mode());
        break;

      case GET_STATUS:
        esp.send_msg(CURRENT_STATUS|esp.get_status());
        break;

      case RESET_MEM:
        esp.reset_mem();
        break;

      case TURNOFF:
        esp.turn_off();
        break;

      case EDIT:
      {
        net_config aux =
        {
          .ssid = {0},
          .password = {0},
          .ip = {0},
          .port = 0
        };
        
        byte port_aux[4];

        esp.get_msg(aux.ssid, SSID_LENGTH);
        esp.light_pattern(PROCESSING);
        esp.light_pattern(READY);
        esp.get_msg(aux.password, PASSWORD_LENGTH);
        esp.light_pattern(PROCESSING);
        esp.light_pattern(READY);
        esp.get_msg(aux.ip,4);
        esp.light_pattern(PROCESSING);
        esp.light_pattern(READY);
        esp.get_msg(&aux.port);
        esp.set_config(&aux);
      }
      break;

      default:
        break;
    }
  }else if (status == ERR)
  {
    Serial.println(esp.get_error_code());
    switch (cmd)
    {
      case RESET_MEM:
        esp.reset_mem();
        break;

      case TURNOFF:
        esp.turn_off();
        break;

      case EDIT:
        net_config aux;
        esp.get_msg(aux.ssid, SSID_LENGTH);
        esp.get_msg(aux.password, PASSWORD_LENGTH);

        for (int i = 0; i < 4; i++)
          esp.get_msg(&aux.ip[i]);

        esp.get_msg(&(aux.port));

        esp.set_config(&aux);
        esp.print_config();
        break;

      default:
        break;
    }
  }
  esp.send_update();
}

esp_client::esp_client()
{
  m_status = CON_CONFIG;
  m_mode = READ;
  m_error_code = 0;
  EEPROM.begin(9 + SSID_LENGTH + PASSWORD_LENGTH);

  int addr = 0;

  m_is_configured = EEPROM.read(addr);
  addr +=1;

  if (m_is_configured)
  {
    for (int i = 0; i < SSID_LENGTH; i++)
      m_config.ssid[i] = EEPROM.read(i + addr);

    addr += SSID_LENGTH;

    for (int i = 0; i < PASSWORD_LENGTH; i++)
      m_config.password[i] = EEPROM.read(i + addr);

    addr += PASSWORD_LENGTH;

    for (int i = 0; i < 4; i++)
      m_config.ip[i] = EEPROM.read(i + addr);

    addr +=4;
    m_config.port = 0;
    
    for (int i = 0; i < 4; i++)
    {
      m_config.port *= 10;
      m_config.port += EEPROM.read(i + addr); 
    }
    
  } else {
    for (int i = 0; i < SSID_LENGTH; i++)
      m_config.ssid[i] = '\0';

    for (int i = 0; i < PASSWORD_LENGTH; i++)
      m_config.password[i] = '\0';

    for (int i = 0; i < 4; i++)
      m_config.ip[i] = 0;

    m_config.port = 0;
  }
}

int esp_client::wifi_connect()
{
  WiFi.begin((char*)m_config.ssid, (char*)m_config.password);
  Serial.println("WiFi Connecting...");

  int t1 = millis();

  while (WiFi.status() != WL_CONNECTED)
  {
    this->light_pattern(CON_CONFIG);
    if ((millis() - t1) > TIMEOUT)
    {
      set_error(1);
      return -1;
    }
    delay(20);
  }
  this->light_pattern(READY);
  delay(20);
  Serial.println("WiFi Connected!");
  return 0;
}

int esp_client::server_connect()
{
  this->connect(m_config.ip, m_config.port);
  Serial.print("Connecting to Server:");
  for(int i = 0; i<4; i++)
    Serial.print(m_config.ip[i]);
  Serial.print(m_config.port);
  Serial.print("Connecting to Server...");
  int t1 = millis();
  while (!this->connected())
  {
    this->light_pattern(CON_CONFIG);
    if ((millis() - t1) > TIMEOUT)
    {
      set_error(2);
      return -1;
    }
  }
  Serial.println("Server Connected!");
  this->light_pattern(READY);
  m_status = READY;
  byte aux;
  get_msg(&aux);
  send_update();
  return 0;
}

int esp_client::serial_connect()
{
   m_status = CON_CONFIG;
    if (Serial)
    {
      esp.wait_serial();
      esp.set_status(READY);
      esp.light_pattern(READY);
      esp.send_update();
      return 0;
    }else
    {
      return -1;
    }
     m_status = READY;
}

void esp_client::turn_off()
{
  this->light_pattern(ASLEEP);
  this->stop();
  this->set_status(ASLEEP);
  esp.send_update();
  delay(3000);
  ESP.deepSleep(0);
}

void esp_client::reset_mem()
{
  if (EEPROM.read(0) != 0)
    EEPROM.write(0, 0);
  EEPROM.commit();
}

byte esp_client::get_status() {
  return m_status;
}

byte esp_client::get_mode() {
  return m_mode;
}

int esp_client::get_msg(byte* msg, int size)
{
  if (msg == NULL)
    return -1;

  if (!this->connected())
  {
    if (!Serial)
    {
      set_error(3);
      return -1;
    }
    for (int i = 0; i < size; i++)
    {
      while (!Serial.available())
        delay(20);
      
      msg[i] = Serial.read();
      
      if (msg[i] == '\n')
      {
        msg[i] = '\0';
        break;
      }
    }
    while (Serial.available())
    {
      yield();
      Serial.read();
    }
  }else
  {
    for (int i = 0; i < size; i++)
    {
      while (!this->available())
        delay(20);
      msg[i] = read();
    }
  }

  return 0;
}

int esp_client::get_msg(int* msg)
{
  if (msg == NULL)
    return -1;

  byte aux[4];

  this->get_msg(aux, 4);
  *msg = 0;

  for(int i = 0; i<4; i++)
  {
    (*msg)*=10;
    (*msg) += aux[i]-'0';
  } 

  return 0;
}

int esp_client::get_msg(byte* msg)
{
  if (msg == NULL)
    return -1;

  if (!this->connected())
  {
     if (!Serial)
    {
      set_error(3);
      return -1;
    }
    while (!Serial.available())
      delay(20);
      
    Serial.read(msg,1);
  } else {
    Serial.println("getting");
    while (!available())
      delay(20);
    *msg = read();
  }
  return 0;
}

int esp_client::get_config(net_config* cfg)
{
  if (cfg == NULL)
    return -1;

  for (int i = 0; i < 11; i++)
  {
    cfg->ssid[i] = m_config.ssid[i];
    cfg->password[i] = m_config.password[i];

    if (i < 4)
      cfg->ip[i] = m_config.ip[i];
  }

  cfg->port = m_config.port;
}

int esp_client::send_msg(byte* msg, int size)
{

  if (msg == NULL)
    return -1;

  if (!this->connected())
  {
    if (!Serial)
    {
      set_error(3);
      return -1;
    }
    Serial.write(msg, size);
  
  } else
  {
    for(int i =0; i<size; i++)
      this->write(msg[i]);
  }
  return 0;
}

int esp_client::send_msg(byte msg)
{
  if (!this->connected())
  {
    if (!Serial)
    {
      set_error(3);
      return -1;
    }
    Serial.write(msg);
  } else
  {
    this->write(msg);
  }
  return 0;
}

bool esp_client::is_configured() {return m_is_configured;}

void esp_client::set_mode(byte mode){m_mode = mode;}

void esp_client::set_status(byte status){m_status = status;}

int esp_client::set_config(net_config* nc)
{
  if (nc == NULL)
    return -1;

  m_is_configured = true;

  for (int i = 0; i < SSID_LENGTH; i++)
    m_config.ssid[i] = nc->ssid[i];

  for (int i = 0; i < PASSWORD_LENGTH; i++)
    m_config.password[i] = nc->password[i];

  for (int i = 0; i < 4; i++)
    m_config.ip[i] = nc->ip[i];

  m_config.port = nc->port;
  int addr = 0;

  if (m_is_configured != EEPROM.read(addr))
    EEPROM.write(addr, (char)m_is_configured);

  addr += 1;

  for (int i = 0; i < 11; i++)
  {
    if (m_config.ssid[i] != EEPROM.read(i + addr))
      EEPROM.write(i + addr, m_config.ssid[i]);
  }
  addr += SSID_LENGTH;

  for (int i = 0; i < 11; i++)
  {
    if (m_config.password[i] != EEPROM.read(i + addr))
      EEPROM.write(i + addr, m_config.password[i]);
  }
  addr+=PASSWORD_LENGTH;

  for (int i = 0; i < 4; i++)
  {
    if (m_config.ip[i] != EEPROM.read(i + addr))
      EEPROM.write(i + addr, m_config.ip[i]);
  }
  addr+=4;

  byte aux[4] = {0};
  int port_aux = m_config.port;
  for(int i = 3; i>=0; i--)
  {
    aux[i] = port_aux%10;
    port_aux/=10;
  }

  for (int i = 0; i < 4; i++)
  {
    if (aux[i] != EEPROM.read(i + addr))
      EEPROM.write(i + addr, aux[i]);
  }
 
  EEPROM.commit();
  EEPROM.end();
  return 0;
}

void esp_client::print_config()
{
  Serial.println((char*)m_config.ssid);
  Serial.println((char*)m_config.password);

  for (int i = 0; i < 4; i++)
    Serial.print(m_config.ip[i]);

  Serial.println();
  Serial.println((char*)m_config.port);
}

void esp_client::light_pattern(byte status)
{
  switch (status)
  {
    case CON_CONFIG:
      digitalWrite(BLUE_PIN, HIGH);
      digitalWrite(RED_PIN, HIGH);
      digitalWrite(GREEN_PIN, LOW);
      delay(100);
      digitalWrite(BLUE_PIN, LOW);
      digitalWrite(RED_PIN, LOW);
      digitalWrite(GREEN_PIN, LOW);
      delay(100);
      break;

    case READY:
      digitalWrite(BLUE_PIN, LOW);
      digitalWrite(RED_PIN, LOW);
      digitalWrite(GREEN_PIN, HIGH);
      break;

    case PROCESSING:
      digitalWrite(GREEN_PIN, LOW);
      digitalWrite(RED_PIN, LOW);
      digitalWrite(BLUE_PIN, HIGH);
      delay(50);
      digitalWrite(BLUE_PIN, LOW);
      delay(50);
      break;

    case ERR:
      digitalWrite(GREEN_PIN, LOW);
      digitalWrite(RED_PIN, HIGH);
      digitalWrite(BLUE_PIN, LOW);
      break;

    case ASLEEP:
      digitalWrite(GREEN_PIN, HIGH);
      digitalWrite(RED_PIN, LOW);
      digitalWrite(BLUE_PIN, HIGH);
      break;
  }
}

void esp_client::set_error(int code)
{
  m_error_code = code;
  this->set_status(ERR);
  this->light_pattern(ERR);
}

int esp_client::get_error_code(){return m_error_code;}

esp_reader::esp_reader(int ss_pin, int rst_pin): MFRC522(ss_pin, rst_pin)
{

  //Serial peripheral interface: para conectar o ESP ao MRFC522.
  SPI.begin();
  //Constrói o RFID I/O
  this->PCD_Init();

  /*Inicializa as chaves como tendo 0xFF em cada bit. Toda tag possui esse
    padrão de fábrica.*/
  for (byte i = 0; i < 6; i++)
  {
    m_key.keyByte[i] = 0xFF;
  }

};

int esp_reader::get_tag_data(byte data[TAG_LENGTH])
{
  //Se não houver nenhum cartão novo presente ou se nenhum cartão puder ser lido, a função não ocorre.
  if (!this->PICC_IsNewCardPresent())
  {
    return -1;
  }
  if (!this->PICC_ReadCardSerial()) return -1;

  //Vetor que armazenará o bloco lido.
  byte buf[18] = {0x0};
  byte sz = 18;

  //Setor do bloco lido. Foi escolhido o segundo (0-indexado) por conveniência.
  byte sector = 1;

  //O número do bloco (cada setor possui 4 blocos, como o setor 1 é o segundo, este é o quarto bloco).
  byte blockAddr = 4;
  //Um bloco especial que armazena configurações de segurança.
  byte trailerBlock = 7;

  //Variável de controle para as leituras.
  MFRC522::StatusCode sts;

  this->light_pattern(PROCESSING);
  this->light_pattern(READY);

  //Tenta autenticar a chave de leitura. Se não for possível, retorna nulo.
  sts = this->PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, trailerBlock, &m_key, &(this->uid));

  if (sts !=  MFRC522::STATUS_OK) {
    return -1;
  }

  //Lê o bloco blockAddr e armazena o resultado no vetor buf.
  sts = (MFRC522::StatusCode) this->MIFARE_Read(blockAddr, buf, &sz);

  //Se a leitura não ocorreu, imprime um erro. Se ocorreu, imprime os bytes armazenados em buf.
  if (sts != MFRC522::STATUS_OK) {
    this->set_error(5);//ERRO de leitura
    return -1;
  }

  //Para a leitura e cancela a autenticação.
  if(this->PICC_HaltA()!=STATUS_OK)
    Serial.println("Falha");
  this->PCD_StopCrypto1();

  //Escreve o código lido no buffer
  for (int i = 0; i < 16; i++)
  {
    Serial.print(buf[i]);
    data[i] = buf[i];
  }

  return 0;
}

int esp_reader::set_tag_data(byte data[16])
{
  //Se não houver nenhum cartão novo presente ou se nenhum cartão puder ser lido,retorna -1
  if (!this->PICC_IsNewCardPresent()) return -1;
  if (!this->PICC_ReadCardSerial()) return -1;

  //Setor do bloco de escrita.
  byte sector = 1;

  //O número do bloco de escrita.
  byte blockAddr = 4;

  //O número do bloco de segurança.
  byte trailerBlock = 7;

  //Variável de controle para as escritas.
  MFRC522::StatusCode sts;

  this->light_pattern(PROCESSING);
  this->light_pattern(READY);

  //Tenta autenticar a chave de escrita. Se não for possível, retorna -1.
  sts = this->PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_B, trailerBlock, &m_key, &(this->uid));

  if (sts !=  MFRC522::STATUS_OK) {
    return -1;
  }

  //Tenta escrever no bloco. Se não for possível, retorna -1.
  sts = (MFRC522::StatusCode) this->MIFARE_Write(blockAddr, data, 16);

  if (sts != MFRC522::STATUS_OK) {
    this->set_error(6);//Ero de escrita
    return -1;
  }

  //Para a leitura e cancela a autenticação.
  this->PICC_HaltA();
  this->PCD_StopCrypto1();

  //Retorna 0 se a escrita foi bem-sucedida.
  return 0;
}

void process_itens()
{
  byte code[16];
  byte cmd = 0;

  esp.set_status(PROCESSING);

  esp.send_msg(esp.get_status());

  while (esp.get_tag_data(code) == -1)
  {
    delay(20);
    yield();
  }

  esp.send_msg(NEW);
  esp.send_msg(code,16);

  while(esp.get_tag_data(code) != -1)
  {
    esp.send_msg(NEW);
    esp.send_msg(code,16);
    delay(20);
    yield();
  }
  esp.send_msg(END);
  esp.set_status(READY);
}

void register_itens()
{
  byte code[TAG_LENGTH] = {0};
  byte cmd = 0;

  esp.set_status(PROCESSING);
  esp.send_msg(esp.get_status());

  while (esp.get_msg(&cmd) != -1 && cmd == CONTINUE)
  {
    esp.get_msg(code, 16);

    while (esp.set_tag_data(code) == -1)
    {
      delay(20);
    }
 
    esp.send_msg(OP_SUCCESS);
  }
  esp.set_status(READY);
}

void esp_client::wait_serial()
{
  while (!Serial.available())
  {
    light_pattern(CON_CONFIG);
    delay(100);
    yield();
  }
  Serial.read();
}

void esp_client::send_update()
{
  send_msg(CURRENT_STATUS|esp.get_status());
  send_msg(CURRENT_MODE|esp.get_mode());
