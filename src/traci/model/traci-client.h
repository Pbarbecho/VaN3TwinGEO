/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

/*
 * Copyright (c) 2018 TU Dresden
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors: Patrick Schmager <patrick.schmager@tu-dresden.de>
 *          Sebastian Kuehlmorgen <sebastian.kuehlmorgen@tu-dresden.de>
 */

#ifndef TRACI_H
#define TRACI_H

#include <map>
#include <vector>
#include <string>
#include <functional>

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"

#include "sumo-TraCIAPI.h"
#include "sumo-TraCIDefs.h"

#include "ns3/vehicle-visualizer.h"

#include "ns3/StationType.h"

#include "ns3/sionna-connection-handler.h"

#define STARTUP_FCN std::function<Ptr<Node>(std::string,TraciClient::StationTypeTraCI_t)>
#define SHUTDOWN_FCN std::function<void(Ptr<Node>,std::string)>

namespace ns3 {

/**
 * Instantánea del estado de un vehículo SUMO en el último paso de
 * sincronización, recibida por SUSCRIPCIÓN TraCI (llega dentro de la
 * respuesta de simulationStep, sin round-trips por vehículo). Es la fuente de
 * datos de VDPTraCI, SUMOSensor y MetricSupervisor: antes cada uno pedía por
 * socket posición/velocidad/rumbo/... por vehículo y por CAM (5-10 round-trips
 * por vehículo cada 100 ms; el sensor y el supervisor, O(N²)).
 *
 * lon/lat se convierten UNA vez por instantánea y solo si alguien las pide
 * (convertXYtoLonLat sí es un round-trip; SUMO hace la proyección).
 */
struct VehicleSnapshot
{
  double x = 0.0, y = 0.0;        // posición SUMO (m)
  double speed = 0.0;             // m/s
  double angle = 0.0;             // rumbo SUMO (0 = N, horario)
  double accel = 0.0;             // m/s²
  double distance = 0.0;          // odómetro (m)
  std::string roadId;             // arista actual
  int laneIndex = 0;
  bool geoValid = false;          // lon/lat ya convertidas para esta instantánea
  double lon = 0.0, lat = 0.0;
  int64_t stamp = -1;             // Simulator::Now() (ns) de la instantánea
};

class TraciClient : public TraCIAPI, public Object
{
public:
  typedef enum {
    StationTypeTraci_vehicle,
    StationTypeTraci_roadSideUnit,
    StationTypeTraci_pedestrian,
    StationTypeTraci_other,
    StationTypeTraci_unspecified
  } StationTypeTraCI_t;

  // register this type with the TypeId system.
  static TypeId GetTypeId (void);

  // constructor and destructor
  TraciClient (void);
  ~TraciClient(void);

  // start up sumo; pass function pointers for including and excluding node functions
  void SumoSetup(STARTUP_FCN includeNode, SHUTDOWN_FCN excludeNode);

  void SumoStop();

  // get associated sumo vehicle for ns3 node
  std::string GetVehicleId(Ptr<Node> node);

  uint32_t GetVehicleMapSize(); // size of vehicle map

  std::vector<std::string> getVehicleNodeMapIds(); // get all vehicle node ids

  std::map< std::string, std::pair< StationType_t, Ptr<Node> > > get_NodeMap() {return m_NodeMap;};
  // referencia (sin copiar el mapa: get_NodeMap copiaba N entradas por llamada)
  const std::map< std::string, std::pair< StationType_t, Ptr<Node> > >& get_NodeMapRef() const {return m_NodeMap;};

  void AddStation(std::string id, float x, float y, float z, Ptr<Node> node);

  std::string GetStationId(Ptr<Node> node);

  void SetSionnaUp() {m_sionna = true;};

  // --- instantáneas por suscripción (ver VehicleSnapshot) -------------------
  // Puntero a la instantánea del vehículo (nullptr si no está suscrito o las
  // suscripciones están desactivadas: el llamante hace la consulta directa).
  const VehicleSnapshot* GetSnapshot (const std::string& vehId);
  // Igual, pero garantizando lon/lat (1 round-trip la primera vez por paso).
  const VehicleSnapshot* GetSnapshotGeo (const std::string& vehId);
  // Todas las instantáneas del paso actual (solo vehículos).
  const std::map<std::string, VehicleSnapshot>& GetSnapshots () const { return m_snapshots; }
  bool UseSubscriptions () const { return m_useSubscriptions; }
  // nº de carriles de una arista, cacheado (constante durante la corrida)
  int GetEdgeLaneNumber (const std::string& edgeId);
  // dimensiones (longitud, anchura) de un vehículo, cacheadas por id
  std::pair<double, double> GetVehicleDims (const std::string& vehId);


private:
  // suscribir las variables de VehicleSnapshot para un vehículo nuevo
  void SubscribeVehicle (const std::string& vehId);
  // volcar los resultados de suscripción del último simulationStep en m_snapshots
  void RefreshSnapshots ();

  bool m_useSubscriptions = true;
  std::map<std::string, VehicleSnapshot> m_snapshots;
  std::map<std::string, int> m_edgeLanes;
  std::map<std::string, std::pair<double, double>> m_vehDims;
  // perform sumo simulation for a certain time step
  void SumoSimulationStep(void);

  // get current positions from sumo vehicles and update corresponding ns3 nodes positions
  void UpdatePositions(void);

  // get new (departed) and removed (arrived) vehicles from sumo
  void GetSumoVehicles(std::vector<std::string>& sumoVehicles);

  // synchronise ns3 nodes with sumo vehicles
  void SynchroniseNodeMap(void);

  // build command line string for sumo start up
  std::string GetSumoCmdString (void);

  // map every sumo vehicle/pedestrian to a ns3 node
  std::map< std::string, std::pair< StationType_t, Ptr<Node> > > m_NodeMap;

  // a vehicle is untracked if it is simulated in sumo but not linked to a ns3 node because of an penetration rate < 1.0
  std::vector<std::string> m_untrackedVehicles;

  // function pointers to node include/exclude functions 
  STARTUP_FCN m_includeNode;
  SHUTDOWN_FCN m_excludeNode;

  // port handling functionality for multiple parallel simulations
  static bool PortFreeCheck (uint32_t portNum);
  static uint32_t GetFreePort (uint32_t portNum=10000);

  // simulation specific data members
  std::string m_sumoAddCmdOpt;
  std::string m_sumoCommand;
  std::string m_sumoConfigPath;
  std::string m_sumoBinaryPath;
  uint16_t m_sumoPort;
  bool m_sumoGUI;

  double m_penetrationRate;
  ns3::Time m_synchInterval;
  ns3::Time m_startTime;
  
  bool m_sumoLogFile;
  bool m_sumoStepLog;
  double m_altitude;
  int m_sumoSeed;
  ns3::Time m_sumoWaitForSocket;

  // Flag pedestrians list empty
  bool m_pedlist_empty = true;

  Ptr<vehicleVisualizer> m_vehicle_visualizer;
  std::string m_netns_name;
  void terminateVehicleVisualizer (void);

  bool m_sionna = false;

};

} // end namespace ns3

#endif /* TRACI_H */

