/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*

  Mateus-n00b, Janeiro 2017

  Este script realiza a troca de pacotes entre veiculos, simulando
  o envio das subtarefas a serem executadas nos veiculos substitutos.

  Objetivos concluidos:
      Realizacao da descoberta de surrogates.
      Distancia entre o cliente e os surrogates.

  TODO:
      Algoritmo de escolha de surrogate.
      Enviar dados via socket.

  Version 2.1

  Fevereiro 2017 - Updates

  Tarefas realizadas:
      Algoritmo de escolha de surrogate.
      Enviar dados via socket.
  TODO:
      Limitar a quantidade de offloads no "crivo"

  License GPLv3

 */

#include "ns3/vector.h"
#include "ns3/string.h"
#include "ns3/socket.h"
#include "ns3/double.h"
#include "ns3/config.h"
#include "ns3/log.h"
#include "ns3/command-line.h"
#include "ns3/mobility-model.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/mobility-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-interface-container.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/internet-module.h"
#include "ns3/ns2-mobility-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/flow-monitor-module.h"

#include <iostream>
#include <unistd.h>
#include <ns3/nqos-wifi-mac-helper.h>


#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"

/*
            Important VARS
*/

// // offloading
// #define GOOD_DISTANCE 150.0
// // number of Nodes in the simulation
// #define PI 3.14
// #define numberOfNodes 100
// // Number of surrogates in the simulation
// #define numberOfSurrogates 2
// // Processing time
// // #define sleepTime     29.0566
// // Packets lenght
// #define packetSize     2094/numberOfSurrogates
// // Enable the decision making
// #define comEscolha true


using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("Projeto-Artigo");
/*

                    GLOBAL VARS

*/

// // offloading
double GOOD_DISTANCE =150.0;
//
double PI = 3.14;
// number of Nodes in the simulation
const int numberOfNodes = 100;
// // Number of surrogates in the simulation
const int numberOfSurrogates =2;
// // Processing time
double sleepTime    =0.8641;
// Packets lenght 500=1kB, 1000=1,6KB e 2000=2,4KB
int packetSize     =1024;
// // Enable the decision making
bool comEscolha =true;

// Container principal
NodeContainer c;
// Container dos surrogates
NodeContainer surrogates;
// Container para o cliente
NodeContainer clients;
NodeContainer providers;
NodeContainer bestProviders;
Ipv4InterfaceContainer ips;
int OffloadSuccess=0;

// Isso eh bem chato. Cuidado com o tamanho dessesarmazenaIndexSurrogatess!
int armazenaIndexSurrogates[numberOfNodes];
int armazenaIndexProviders[numberOfNodes];

bool KEY = true;
double tempoIni;
double tempoFin;


/*

                    FUNCTIONS

*/


/*

                  SERVER STUFF

*/
Ptr<Socket> client_side;
Ptr<Socket> server_side;

double timecmp;
double tempoEsperado;

void SendAfter(Ptr<Socket> socket) {
  std::cout << "[SERVER] Sending the tasks..." << std::endl;

  socket->Send(Create<Packet> (packetSize),0);
  socket->Close();
}

void serverHandler(Ptr<Socket> socket) {
  Ptr<Packet> pk = socket->Recv(2014,0);
  std::cout << "[SERVER] Received task\n";
  std::cout << "[SERVER] Processing..." << std::endl;
  std::cout << "[SERVER] Sending the tasks..." << std::endl;
  timecmp = Simulator::Now().GetSeconds();
  tempoEsperado = timecmp+sleepTime;

  std::cerr << "[DEBUG] Time before sleep => "<< Simulator::Now().GetSeconds () << std::endl;
  Simulator::Schedule(Seconds(sleepTime),&SendAfter,socket);
}

// Accept callback
void accept(Ptr<Socket> socket,const ns3::Address& from)
{

    std::cout<<"[SERVER] Connection accepted"<< std::endl;
    socket->SetRecvCallback (MakeCallback (&serverHandler));

}

// The server side
void serverSide(uint32_t index) {
      if (OffloadSuccess > numberOfSurrogates) {
          exit(0);
      }
     TypeId tid = TypeId::LookupByName ("ns3::TcpSocketFactory");
     InetSocketAddress listen = InetSocketAddress (Ipv4Address::GetAny (), 55555);

        std::cout << "[SERVER] Server " << index << " is on duty!" << std::endl;
        server_side = Socket::CreateSocket(providers.Get(index),tid);

        server_side->Bind(listen);
        server_side->Listen();
        server_side->SetAcceptCallback (MakeNullCallback<bool, Ptr<Socket>,const Address &> (),MakeCallback(&accept));
}

/*

              CLIENT STUFF

*/

bool existe(int *my_array, int item, size_t tam){
      for (size_t i = 0; i < tam; i++) {
          if (my_array[i] == item) {
              return true;
          }
      }
      return false;
}


//
void ordenacao(void) {
    Ptr<MobilityModel> model1 = clients.Get(0)->GetObject<MobilityModel>();
    Ptr<MobilityModel> model2 = providers.Get(0)->GetObject<MobilityModel>();

    double menorDis = 99999999.0;
    int menorIndex = 0;
    int menorAnterior[providers.GetN()];
    double temp = 0.0;

    for (size_t j = 0; j < providers.GetN(); j++) {
        for (size_t i = 0; i < providers.GetN(); i++) {
            model2 = providers.Get(i)->GetObject<MobilityModel>();
            temp = model2->GetDistanceFrom(model1);
            if (existe(menorAnterior,i,j)) {
                if (temp < menorDis){
                    menorDis = temp;
                    menorIndex = i;
                }
           }
        }
        std::cout << "[SYSTEM] Adicionado..." << std::endl;
        bestProviders.Add(providers.Get(menorIndex));
        std::cout << "[SYSTEM] Adicionado indice "<< menorIndex << std::endl;
        // providers.Remove(menorIndex);
        menorAnterior[j] = menorIndex;
        menorDis = 99999999.0;
    }
}


void clientHandler(Ptr<Socket> socket) {
  FILE *fp = fopen("/tmp/results", "a+");

  while (socket->Recv(1024,0)) {
      std::cout << "[CLIENT] Received one packet!" << std::endl;
  }
  std::cout << "[CLIENT] Received results!\n";
  std::cerr << "[DEBUG] Time after sleep => "<< Simulator::Now().GetSeconds () << std::endl;
  OffloadSuccess++;
  std::cout << "OffloadSuccess: "<< OffloadSuccess << std::endl;

  if (OffloadSuccess == numberOfSurrogates) {
      tempoFin = Simulator::Now().GetSeconds ();

      std::cout << "[SYSTEM] Offload OK =)" << std::endl;
      std::cout << "[SYSTEM] RTT final: "<< tempoFin-tempoIni << std::endl;
      std::stringstream rtt;
      rtt << tempoFin-tempoIni;

      if (comEscolha) {
          fprintf(fp,"1:CE:%s\n",rtt.str().c_str() );
          exit(0);
      }else{
          fprintf(fp,"1:SE:%s\n", rtt.str().c_str());
    }
    fclose(fp);
}
socket->Close();
}

// The client side
void clientSide(uint32_t index) {
// void clientSide(void) {
    // Address from;
    // FILE *fp = fopen("/tmp/results", "a+");

    TypeId tid = TypeId::LookupByName ("ns3::TcpSocketFactory");
    std::cerr << "\t\t[DEBUG] Starting connection with "<< index << std::endl;
    std::cout << "[SYSTEM] Number of providers: "<< providers.GetN() << std::endl;

    if (providers.GetN() >= numberOfSurrogates){
                if (OffloadSuccess > numberOfSurrogates) {
                    exit(0);
                }

                // for (size_t i = 0; i < providers.GetN(); i++) {
                    std::cout << "Assert: TRUE" << std::endl; // For my purposes
                    InetSocketAddress end = InetSocketAddress (providers.Get(index)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal(), 55555);
                    // InetSocketAddress listen = InetSocketAddress (Ipv4Address::GetAny (), 55555);
                    client_side = Socket::CreateSocket(clients.Get(0),tid);

                    uint32_t status = client_side->Connect(end);
                    std::cout << "[SYSTEM] Connection status => "<< status << std::endl;
                    std::cout << "[CLIENT] Sending tasks..." << std::endl;

                    // Envia o pedaco da matriz
                    client_side->Send(Create<Packet> (packetSize),0);
                    // client_side->Bind(listen);
                    client_side->SetRecvCallback(MakeCallback(&clientHandler));
                // }
    }else{
          std::cout << "[CLIENT] Not enough replies!" << std::endl;
          // Remover depois
          std::cout << "Assert: FALSE" << std::endl;
          exit(-1);
    }
}
/*

                REPLY THE REQUESTS

*/
// Function to receive "hello" packets
void listenForRequests(Ptr<Socket> socket) {
    Address from;

    Ptr<Packet> packet= socket->RecvFrom(from);
    // Show the content
    uint8_t *buff = new uint8_t[packet->GetSize()];
    packet->CopyData(buff,packet->GetSize());

    Ipv4Address ipv4From = InetSocketAddress::ConvertFrom(from).GetIpv4();

    std::string data = std::string((char*)buff);

    std::cout << "[SERVER] Message received: "<< data << std::endl;
    std::cout << "[SERVER] Received request from: "<< ipv4From << std::endl;
    std::cout << "[SERVER] Sending the reply..." << std::endl;
    socket->Connect(InetSocketAddress(ipv4From,80));

    socket->Send(Create<Packet> (60));
    socket->Close();
}

/*

              START THE DISCOVER PHASE

*/

// Send packets
void SendPkt(Ptr<Socket> socket) {
    std::stringstream msgx;
    msgx << "Hello Mateus";
    Ptr<Packet> pkt = Create<Packet>((uint8_t*) msgx.str().c_str(), packetSize);
    socket->Send(pkt);
}

/*

              USEFUL GETs

*/
// Returns the speed of a node
double GetSpeed(Ptr<const MobilityModel> model1){
      return model1->GetVelocity().x;
}

// Get distance from a node1 to a node2
double GetDistance(Ptr<const MobilityModel> model1,Ptr<const MobilityModel> model2){
        return model1->GetDistanceFrom (model2);
}

// Get the angle to retrieve the direction
int getAngle(Ptr<const MobilityModel> model1){
    return atan2(model1->GetVelocity().y,model1->GetVelocity().x)*180/PI;
}

bool sameDirection(Ptr<const MobilityModel> model1,Ptr<const MobilityModel> model2){
     // Moving to the east
     if (getAngle(model1) == getAngle(model2)) {
          return true;
     }
     else{
        return false;
     }
}

/*

            CALCULATE THE LIFETIME OF THE LINK

*/
// Alisson's function
double linkEstimatedLifeTime(Vector my_velocity ,Vector  neighbour_velocity,
  Vector my_position, Vector neighbour_position, double range=200.0){

  double let, a, b, c, x_1, x_2, delta;
  //se os carros estão parados
  if ((my_velocity.x == 0.0) && (my_velocity.y == 0.0) && (neighbour_velocity.x == 0.0)
      && (neighbour_velocity.y == 0.0)) {
    let = 100.0;
  } else { //pelo menos um dos carros está em movimento
    if (my_velocity.x - neighbour_velocity.x == 0.0) { neighbour_velocity.x = neighbour_velocity.x + 0.00001; }
    if (my_velocity.y - neighbour_velocity.y == 0.0) { neighbour_velocity.y = neighbour_velocity.y + 0.00001; }
    if (my_position.x - neighbour_position.x == 0.0) { neighbour_position.x = neighbour_position.x + 0.00001; }
    if (my_position.y - neighbour_position.y == 0.0) { neighbour_position.y = neighbour_position.y + 0.00001; }

    a = pow((my_velocity.x - neighbour_velocity.x), 2.0) +
        pow((my_velocity.y - neighbour_velocity.y), 2.0);
    b = 2*( (my_position.x - neighbour_position.x)*(my_velocity.x - neighbour_velocity.x)
        +(my_position.y - neighbour_position.y)*(my_velocity.y - neighbour_velocity.y)
    );
    c = pow( (my_position.x - neighbour_position.x) , 2.0) +
        pow( (my_position.y - neighbour_position.y) ,2.0) - pow(range, 2.0);
    if (a == 0.0)	{a = 0.000000001;}
    delta = pow(b, 2.0) - (4.0*a*c);


    if (delta > 0.0) {
      x_1 = (-b + pow( delta , 0.5))/(2.0*a);
      x_2 = (-b - pow( delta , 0.5))/(2.0*a);
      if (x_1 > 0.0) {
        let = x_1;
      } else if (x_2 > 0.0) {
        let = x_2;
      }
    } else if (delta == 0.0) {
      let = (-b)/(2.0*a);
    } else {
      let = 100.0;
    }
  }

  //limite superior do let. no caso de os veículos terem mobilidades semelhantes.
  //exemplo: dois carros se movendo na mesma direção e com as mesmas velocidades.
  if(let > 100.0) { let = 100.0; }

  return let;
}

// Receive the reply packets and make the surrogate's choice
void RecRepPkt(Ptr<Socket> socket) {
  Address from;
  Ptr<Packet> packet= socket->RecvFrom(from);
  Ipv4Address ipv4From = InetSocketAddress::ConvertFrom(from).GetIpv4();
  std::cout << "[CLIENT] Received reply from: "<< ipv4From << std::endl;

  for (size_t i = 0; i < surrogates.GetN(); i++) {
      if (ipv4From == surrogates.Get(i)->GetObject<Ipv4>()->GetAddress (1, 0).GetLocal()){

          std::cerr << "[DEBUG] The surrogate index is => "<< i << std::endl;

          Ptr<MobilityModel> model1 = clients.Get(0)->GetObject<MobilityModel>();
          Ptr<MobilityModel> model2 = surrogates.Get(i)->GetObject<MobilityModel>();

          // To use in the 'lte' function
          Vector my_velocity = model1->GetVelocity();
          Vector neighbour_velocity = model2->GetVelocity();
          Vector my_position = model1->GetPosition();
          Vector neighbour_position = model2->GetPosition();

          // Returns the distance between the client and the surrogates
          double distance = GetDistance(model1,model2);

          double mySpeed = GetSpeed(model1);
          double surrogateSpeed = GetSpeed(model2);

          std::cout << "\n[SYSTEM] Distance from client to server "<< ips.GetAddress(armazenaIndexSurrogates[i]) << " == "
          << distance
          << std::endl;
          std::cout << "[SYSTEM] Speed of the client: " << mySpeed << std::endl;
          std::cout << "[SYSTEM] Speed of the surrogate: "<< surrogateSpeed << std::endl;
          // std::cout << "[SYSTEM] Address "<< surrogates.Get(i)->GetObject<Ipv4>()->GetAddress (1, 0).GetLocal() << std::endl;
          // std::cout << "POS X (client): "<< model1->GetPosition().x << std::endl;
          // std::cout << "POS X: (surrogate): "<< model2->GetPosition().x << std::endl;
          // std::cout << "POS Y (client): "<< model1->GetPosition().y << std::endl;
          // std::cout << "POS Y: (surrogate): "<< model2->GetPosition().y << std::endl;
          // std::cout << "Angle (client): "<< getAngle(model1) << std::endl;
          // std::cout << "Angle (surrogate): "<< getAngle(model2) << std::endl;
          std::cout << "Tempo de vida do enlace: "<< linkEstimatedLifeTime(my_velocity,neighbour_velocity,my_position,neighbour_position) << std::endl;

          // Desicion making
          if (comEscolha) {
              // Se o switch estiver on
              // Tempo de vida do enlace 'let'
              double let = linkEstimatedLifeTime(my_velocity,neighbour_velocity,my_position,neighbour_position);

              // O valor 0.05 e uma margem utilizada para o RTT medio
              if (let >= sleepTime+0.05){ //&& providers.GetN() <= numberOfSurrogates){
                // Para realizar mais Offloads deve-se zerar oarmazenaIndexSurrogates e reiniciar o processo
                      providers.Add(surrogates.Get(i));

                      std::cerr << "\n[SYSTEM] Surrogate " << i << " Address => "<< ips.GetAddress(armazenaIndexSurrogates[i]) << " choosed! (By decision making)\n" << std::endl;
                      std::cout << "\n[SYSTEM] numberOfSurrogates "<< numberOfSurrogates<< std::endl;

              }

          }else{
              // Se o switch estiver off
              if (providers.GetN() <= numberOfSurrogates) {
                  providers.Add(surrogates.Get(i));
                  std::cerr << "\n[SYSTEM] Surrogate " << i << " Address => "<< ips.GetAddress(armazenaIndexSurrogates[i]) << " choosed! (random way)\n" << std::endl;

              }
          }
      }
   }

   if (providers.GetN() == numberOfSurrogates) {
        for (size_t i = 0; i < providers.GetN(); i++) {
             std::cerr << "[DEBUG] Index: "<< i << std::endl;
             Simulator::Schedule(Seconds(0),serverSide,i);
             Simulator::Schedule(Seconds(0),clientSide,i);
        }
   }
}

/*

            STARTS THE SERVER PHASE

*/
void sendRep(uint32_t index) {
      InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
      TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

      Ptr<Socket> sink = Socket::CreateSocket (c.Get (index), tid);
      sink->Bind(local);
      sink->SetRecvCallback(MakeCallback(&listenForRequests));
}

// Pega o tempo de inicio do processo
void GetTempoIni() {
    tempoIni =  Simulator::Now().GetSeconds ();
    std::cout << "[DEBUG] Tempo inicial eh: "<< tempoIni << std::endl;
}

static void
CourseChange (std::ostream *os, std::string foo, Ptr<const MobilityModel> mobility)
{
  Vector pos = mobility->GetPosition (); // Get position
  Vector vel = mobility->GetVelocity (); // Get velocity

  // Prints position and velocities
  *os << Simulator::Now () << " POS: x=" << pos.x << ", y=" << pos.y
      << ", z=" << pos.z << "; VEL:" << vel.x << ", y=" << vel.y
      << ", z=" << vel.z << std::endl;
}

/*
          MAIN FUNCTION
*/

int main (int argc, char *argv[])
{

  ns3::PacketMetadata::Enable ();
  std::string phyMode ("OfdmRate6MbpsBW10MHz");

  double interval = 1.0; // seconds

  srand(time(NULL));

  std::string traceFile;
  std::string logFile;

  // Enable logging from the ns2 helper
  // LogComponentEnable ("Ns2MobilityHelper",LOG_LEVEL_DEBUG);

  // Parse command line attribute
  CommandLine cmd;
  cmd.AddValue ("traceFile", "Ns2 trace de mobilidade", traceFile);
  cmd.AddValue ("logFile", "arquivo de log de mobilidade", logFile);
  cmd.AddValue ("packetSize", "tamanho do pacote", packetSize);
  cmd.AddValue ("sleepTime", "tempo de processamento", sleepTime);
  cmd.AddValue ("comEscolha", "Habilita o algoritmo", comEscolha);
  cmd.Parse (argc,argv);
  if (comEscolha) {
      std::cerr << "[DEBUG] comEscolha ativado" << std::endl;
  }
  packetSize = packetSize/numberOfSurrogates;

  // Check command line arguments
  if (traceFile.empty () || logFile.empty ())
    {
      std::cout << "Usage of " << argv[0] << " :\n\n"
      "./waf --run \"vanets_tcp"
      " --traceFile=src/mobility/examples/default.ns_movements"
      " --logFile=ns2-mob.log\" \n\n"
      "NOTE: ns2-traces-file could be an absolute or relative path. You could use the file default.ns_movements\n"
      "      included in the same directory of this example file.\n\n"
      "NOTE 2: Number of nodes present in the trace file must match with the command line argument and must\n"
      "        be a positive number. Note that you must know it before to be able to load it.\n\n"
      "NOTE 3: Duration must be a positive number. Note that you must know it before to be able to load it.\n\n";

      return 0;
    }


  std::ofstream os;
  os.open (logFile.c_str ());
  Time interPacketInterval = Seconds (interval);

  // Create the nodes
  c.Create(numberOfNodes);

  /*

                      MOBILITY

  */

// Habilitando a mobilidade do ns2 que utiliza os traces gerados pelo SUMO.
MobilityHelper mobility;
Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile);

// Instalando a mobilidade nos nodes
  ns2.Install ();
  mobility.Install (c);


  // Set the TCP Segment Size
  Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue (1500));

  Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange",
                   MakeBoundCallback (&CourseChange, &os));

  /*

                      PHY LAYER

  */

  // Set of helpers will help us to put together the wifi NICs we want
  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::NakagamiPropagationLossModel");
  Ptr<YansWifiChannel> channel = wifiChannel.Create ();
  wifiPhy.SetChannel (channel);

  // ns-3 supports generate a pcap trace
  wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);
  // wifiPhy.Set ("TxGain", DoubleValue(15) );
  // wifiPhy.Set ("RxGain", DoubleValue (5) );
  //
  // wifiPhy.Set("EnergyDetectionThreshold",DoubleValue(-100.0));


  /*

                      MAC LAYER

  */

  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();

  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue (phyMode),
                                      "ControlMode",StringValue (phyMode));
  NetDeviceContainer devices = wifi80211p.Install (wifiPhy, wifi80211pMac, c);



  /*

            INTERNET STACK

  */
  // Instalando a pilha de protocolos
  InternetStackHelper internet;
  internet.Install (c);
  /*

            DHCP SERVICE

  */
  // DHCP service
  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  ips = ipv4.Assign (devices);

  /*

            CREATING SOCKETS

  */
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

  uint32_t the_client = rand() % numberOfNodes + 0;
  std::cout << "\t\t[SYSTEM] The client ==> "<< ips.GetAddress(the_client) << std::endl;

  // Crio socket cliente
  Ptr<Socket> client = Socket::CreateSocket (c.Get (the_client), tid);
  clients.Add(c.Get(the_client));

  // Usado para receber as respostas dos servers
  InetSocketAddress port = InetSocketAddress (Ipv4Address::GetAny (), 80);

  // Usado para enviar os rep/req
  InetSocketAddress broadcast = InetSocketAddress (Ipv4Address("255.255.255.255"), 80);
  client->SetAllowBroadcast(true);

  client->Connect (broadcast);
  client->Bind(port);
  client->SetRecvCallback(MakeCallback(&RecRepPkt));

  // Instalando as applicacoes
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
  // Escolhendo os surrogates
  for (size_t x = 0; x < numberOfNodes; x++) {
      if (x != the_client) {
          surrogates.Add(c.Get(x));
          // Armazeno o index dos surrogates
         armazenaIndexSurrogates[x] = x;

        //  Listen for the requests
          Ptr<Socket> sink = Socket::CreateSocket (c.Get (x), tid);
          sink->Bind(local);
          sink->SetRecvCallback(MakeCallback(&listenForRequests));

    }
  }

  std::cout << "[SYSTEM] Number of surrogates: "<< surrogates.GetN() << std::endl;

  // Animacao gerada para analise
  AnimationInterface anim ("vanets-animation.xml");

  // Habilitando trace
  AsciiTraceHelper ascii;
  wifiPhy.EnableAsciiAll (ascii.CreateFileStream ("resultados.tr"));

  wifiPhy.EnablePcap ("client-80211p", devices.Get(the_client),true);
//  Agendo a execucao da acao
  uint32_t t = rand() % 20 + 10;
  t= 5;
  std::cout << "\t\t[SYSTEM] Execution started at "<< t << " seconds" << std::endl;
  Simulator::Schedule(Seconds(t),GetTempoIni);
  Simulator::Schedule(Seconds(t),SendPkt,client);

  Simulator::Stop (Seconds (200));
  Simulator::Run ();
  Simulator::Destroy ();

  os.close (); // close log file
  return 0;
}
