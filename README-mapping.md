# Robótica Inteligente -- Master ICSI/KISA 2020-2021
## Práctica final: Mapeado laser

Práctica final de la asignatura de Robótica Inteligente.

## Instalación

Tenemos que incluir el repo en `~/icsi/ros/src` para que la configuración del CMakeLists que haya en local lo lea bien.

```
cd ~/icsi/ros/src
git clone git@gitlab.com:r3v1/laser_mapping.git
```

## Descripción

Construir un mapa del entorno mientras se teleopera el robot a través del teclado.
Para ello, hay que proyectar las lecturas del laser en el espacio de coordenadas global en el
que se mueve el robot, haciendo uso de la odometría.

Sin embargo, en vez de teleoperar el robot, aplicamos el sorteo de obstáculos implementado en la [Práctica 1](https://gitlab.com/r3v1/robot_wander).

## Autores

David - `drevillas002@ikasle.ehu.eus`

Mónica - `mramperez001@ikasle.ehu.eu`

