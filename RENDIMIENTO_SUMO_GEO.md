# Parches de rendimiento TraCI (integración SUMO-GEO) — septiembre 2026

Objetivo: que VaN3Twin escale a **cientos de vehículos** sin que el tráfico
TraCI con SUMO sea el cuello de botella. Ver el análisis completo con cifras
en `docs/RENDIMIENTO_2026-09.md` del repo `SUMO_GEO`.

## Qué cambia

| Fichero | Cambio |
|---|---|
| `src/traci/model/traci-client.{h,cc}` | `VehicleSnapshot`: estado de **todos** los vehículos SUMO (posición, velocidad, rumbo, aceleración, odómetro, arista, carril) recibido por **suscripción TraCI** dentro de la respuesta de cada `simulationStep`. Nuevos accesores `GetSnapshot`, `GetSnapshotGeo` (lon/lat convertidas una vez por paso y solo si se piden), `GetSnapshots`, `GetEdgeLaneNumber` y `GetVehicleDims` (cachés), `get_NodeMapRef` (sin copiar el mapa). `UpdatePositions` ya no hace un `getPosition` por vehículo. Atributo `UseSubscriptions` (por defecto `true`; `false` = comportamiento anterior). |
| `src/automotive/model/Facilities/vdpTraci.{h,cc}` | Todos los getters (`getSpeedValue`, `getHeadingValue`, `getTravelledDistance`, `getPosition`, `getPositionXY`, `get{CAM,CPM,MCM}MandatoryData`, `getLanePosition`) leen la instantánea; consulta directa solo si no existe. |
| `src/automotive/model/Facilities/caBasicService.cc` | Las cadenas de depuración de `checkCamConditions` (que hacían 2 `getPosition` + 2 conversiones por vehículo cada 100 ms) solo se construyen con el log de disparo activo. |
| `src/automotive/model/utilities/sumo-sensor.cc` | El sensor SUMO ya no hace `getIDList` + `getPosition` + `convertXYtoLonLat` por **cada** vehículo (O(N²) round-trips cada 100 ms): filtra por distancia euclídea en metros SUMO sobre las instantáneas; dimensiones cacheadas; una conversión lon/lat por objeto detectado. |
| `src/automotive/model/Measurements/MetricSupervisor.cc` | `signalSentPacket` (por paquete TX con `--met-sup`) resolvía la posición de **todos** los nodos por TraCI: ahora usa las instantáneas y una sola conversión del emisor. Peatones y RSU siguen el camino anterior. |
| `src/automotive/examples/v2v-emergencyVehicleAlert-80211p.cc` | Flag `--pcap` (por defecto `true`); `--pcap=false` desactiva los pcap por nodo (volumen ~N²). |

Coste TraCI por vehículo y por intervalo de 100 ms, antes → después:
`UpdatePositions` 1 → 0; comprobación CAM 5-9 → 0; generación CAM 8 → 1;
sensor N+ → 0; MetricSupervisor 2·N por paquete → 1 por paquete.

## Cómo aplicar (contenedor `van3twin`)

Las imágenes nuevas de `van3twin-docker` ya clonan este `master`. Con un
volumen `ns3-workspace` ya poblado, dentro del contenedor:

```bash
cd ~/VaN3Twin
git pull                          # trae estos parches al volumen
cd ns-3-dev && ./ns3 build        # recompila traci + automotive (minutos)
```

## Validación (pendiente en el Mac: en la nube no hay árbol ns-3 para compilar)

1. `./ns3 build` sin errores.
2. EVA de referencia (20 vehículos), comparar con una corrida anterior:
   ```bash
   ./ns3 run "v2v-emergencyVehicleAlert-80211p --sumo-gui=false --met-sup=true --sumo-updates=0.1 --num-traci-clients=2 --sim-time=50"
   ```
   * mismo número de CAM/CPM enviados y recibidos (±1 %), PRR y latencia
     medias equivalentes (el modelo no cambia, solo la fuente de los datos);
   * el visor SUMO-GEO muestra la movilidad y los mensajes igual que antes;
   * `INFO-veh*` con CPM > 0 (el sensor sigue detectando vecinos, incluidos
     los vehículos no equipados: se suscriben TODOS los de SUMO).
3. Escalado: `--mob-trace cars_120.rou.xml --sumo-config .../map_120.sumo.cfg`
   y comparar el tiempo de pared por segundo simulado con y sin
   `--ns3::TraciClient::UseSubscriptions=false`.
4. Si algo falla, `--ns3::TraciClient::UseSubscriptions=false` restaura el
   comportamiento anterior sin recompilar.

## Notas de diseño

* Se suscriben **todos** los vehículos de SUMO (también los no equipados, que
  el sensor debe "ver"), no solo los nodos ns-3. La suscripción se hace en
  `GetSumoVehicles` al detectar la salida; como SUMO empieza a servirla en el
  siguiente `simulationStep`, la primera instantánea se rellena con una
  lectura directa (una vez por vehículo).
* `convertXYtoLonLat` sigue siendo un round-trip (SUMO hace la proyección);
  se hace como mucho una vez por vehículo y paso, y solo si alguien pide
  lon/lat. Para eliminarlo del todo habría que replicar la proyección de la
  red (`<location projParameter>`) en ns-3 con proj.
* Lo que NO cambia y sigue siendo O(N²) por naturaleza: las recepciones
  802.11p (cada trama llega al PHY de todos los nodos del canal Yans) y los
  pcap por nodo. Para flotas grandes: `--pcap=false`, y considerar un canal
  con corte por distancia.
