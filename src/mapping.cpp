#include <iostream>
#include <unistd.h>
#include <csignal>
// Opencv
//#include "opencv2/core/version.hpp"
//#include "opencv2/core/types_c.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

// ROS

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#define UP_ARROW 72
#define DOWN_ARROW 80
#define LEFT_ARROW 75
#define RIGHT_ARROW 77

#define WINC 0.2
#define VINC 0.1
#define VMAX 0.5
#define WMAX 1.0

#define MXSIZE 1000
#define MYSIZE 1000
#define SCALE 50

#define PI 3.141592654
#define RTOD(r) (r * 180.0/PI)
#define DTOR(d) (d * PI/ 180.0)

using namespace cv;
using namespace std;

sensor_msgs::LaserScan rawscan;
geometry_msgs::Twist cmd_vel_msg;
ros::Publisher cmd_vel_pub;

float rx0, ry0, rtheta0;

cv::Mat mymap;

float v = 0, w = 0;

/* La cámara apunta hacia 180º */
double orientacion = 180;

/* Asegurar que se para si no hay camino fácil */
bool parar = false;

/* Parámetros de velocidades */
float vel_lineal = 0.15;
float max_vel_lineal = 0.2;
float min_vel_lineal = 0.05;

// #####################################
// Parte del robot_wander
// #####################################

int camino_mas_ancho(int apertura[], int cierre[], int j) {
    /**
     * Dadas dos listas de enteros (índices) y el número de caminos j (que no
     * tiene que ser igual que la longitud de los caminos), calcula el camino
     * más ancho.
     *
     * Por ejemplo:
     *      apertura = [0, 10, 15, 0, 0, 0]
     *      cierre   = [8, 14, 15, 0, 0, 0]
     *      j        = 3
     *
     * El camino más ancho será el índice:
     *      0
     */
    int max_len = -1;
    int best_idx = -1;
    for (int i = 0; i <= j; i++) {
        int d = cierre[i] - apertura[i];
        if (d > max_len) {
            max_len = d;
            best_idx = i;
        }
    }

    return best_idx;
}

void move(float x, float y, float theta) {
    /**
    * Publica en el topic de velocidad de ROS
    * la velocidad que le queremos dar al bicho
    */
    cmd_vel_msg.linear.x = x;
    cmd_vel_msg.linear.y = y;
    cmd_vel_msg.angular.z = DTOR(theta);
    cmd_vel_pub.publish(cmd_vel_msg);
}

void check() {
    /**
     * Comprueba en qué situación se encuentra el bicho y ejecuta la
     * acción correspondiente:
     *  - Si la bandera `parar` está activa, se para porque no tiene
     *      un camino claro a dónde ir. Luego comienza a girar poco a
     *      poco para buscar un camino y seguir hacia delante.
     *  - En otro caso, seguirá con una velocidad lineal hacia delante
     *
     *  También añadimos un apartado en el que suaviza las velocidades,
     *  haciendo que si ha estado parado, incremente poco a poco la
     *  velocidad lineal y por el contrario, si tiene que realizar un giro
     *  muy pronunciado, reduzca la velocidad lineal poco a poco.
     */
    // Establece la velocidad angular en función de lo mucho
    // que tenga que girar
    float vel_ang = orientacion - 180;

    if (parar) {
        vel_ang = 25;
        vel_lineal = 0;
        printf("[PARADO] ");
    } else {
        /* Suavizar los cambios de velocidad */

        // Evitar que la velocidad mínima no sea 0
        if (vel_lineal < min_vel_lineal) vel_lineal = min_vel_lineal;

        // Si el giro es muy pronunciado, reduce la velocidad lineal
        if (abs(vel_ang) > 15) {
            vel_lineal = vel_lineal * 0.95;
            if (vel_lineal < min_vel_lineal) vel_lineal = min_vel_lineal;
        } else {
            vel_lineal = vel_lineal * 1.15;
            if (vel_lineal > max_vel_lineal) vel_lineal = max_vel_lineal;
        }
    }

    // Publica la información en el topic de velocidad
    move(vel_lineal, 0, vel_ang);
    printf("Velocidad angular %.2f - Velocidad lineal: %.2f\n", vel_ang, vel_lineal);
}

// #####################################
// Parte del mapping
// #####################################

int setObstacle(float xpos, float ypos, cv::Vec3b color) {
    /**
     * Simplemente `pinta` un píxel de la matriz con un color
     */
    int i, j;
    int MX0 = MXSIZE / 2;
    int MY0 = MYSIZE / 2;
    i = MX0 + xpos * SCALE;
    j = MY0 + ypos * SCALE;

    if (i < 0 || i > MXSIZE) {
        //std::cerr << "X size exceeded" << std::endl;
        return -1;
    }
    if (j < 0 || j > MYSIZE) {
        //std::cerr << "y size exceeded" << std::endl;
        return -1;
    }
    // mymap.at<char>(i, j) = color;
    mymap.at<cv::Vec3b>(cv::Point(j, i)) = color;
    return 1;
}

void
processOdom(const nav_msgs::Odometry::ConstPtr &odom) {
    /**
     * Escanea los 180º delanteros del robot y establece
     * la posición de cada obstaćulo en el mapa
     *
     * Opcionalmente, se puede descomentar las líneas de
     * `diversión` para ver cómo se difumina el mapa con el
     * paso del tiempo
     */
    float rtheta, I;
    float rx, ry, ltheta;
    int i;

    // Sólo por diversión: Harry Potter y el prisionero de Azkaban
    //  for (int x = 0; x < MXSIZE; x++) {
    //  	for (int y = 0; y < MYSIZE; y++) {
    //  		mymap.at<cv::Vec3b>(cv::Point(y, x)) = mymap.at<cv::Vec3b>(cv::Point(y, x)) - cv::Vec3b(12, 0, 1);
    //  	}
    //  }

    rtheta = tf::getYaw(odom->pose.pose.orientation);
    rx = odom->pose.pose.position.x;
    ry = odom->pose.pose.position.y;
    setObstacle(rx, ry, cv::Vec3b(0, 0, 255));
    //printf("Size: %d \n", rawscan.ranges.size());
    for (i = 0; i < rawscan.ranges.size(); i++) {
        // Solo los 180º de delante, parece que da error de otra manera
        if (i > 90 && i < 270) {
            I = rawscan.ranges[i];
            if (I < 5) {
                ltheta = -3.12413907051 + i * 0.0174532923847;
                float ox = rx + I * cos(rtheta + ltheta);
                float oy = ry + I * sin(rtheta + ltheta);
                // Set the obstacle in the map
                setObstacle(ox, oy, cv::Vec3b(255, 0, 0));
            }
        }
    }
    cv::imshow("MAP", mymap);
    cv::waitKey(1);
}

void
readLaser(const sensor_msgs::LaserScan::ConstPtr &scan) {
    /**
     * Lee los datos del laser de la tortuga y adapta la velocidad
     *
     * Nos interesan los índices 90 al 270
     *
     *                  Cámara
     *                   180º
     *               , - ~ ~ ~ - ,
     *           , '               ' ,
     *         ,                       ,
     *        ,                         ,
     *       ,                           ,
     *  270º ,                           , 90º
     *       ,                           ,
     *        ,                         ,
     *         ,  ___________________  ,
     *          `   = @ == === @ ==  ,'
     *            \ ________________/
     *                  Puertos
     *                     0º
     */
    rawscan = *scan;

    // Definimos un umbral mínimo
    double umbral = 1;
    bool caminos[180];

    // Obtener una subsecuencia de trues
    int empieza_camino[90] = {0};
    int acaba_camino[90] = {0};
    int j = 0;

    // Generar el histograma que nos muestra qué caminos seguir
    for (int i = 90; i <= 270; i++) {
        int idx = i - 90;
        bool hay_camino = scan->ranges[i] > umbral;
        caminos[idx] = hay_camino;

        // Comprobar la apertura
        if (i == 90 && hay_camino) empieza_camino[j] = idx;
        else {
            if (hay_camino && !caminos[idx - 1]) {
                empieza_camino[j] = idx;
            }
        }
        // Comprobar el cierre
        if (i > 90) {
            if (!hay_camino && caminos[idx - 1]) {
                acaba_camino[j] = idx - 1;
                j++;
            }
        }
        if ((i == 270) && hay_camino) acaba_camino[j] = idx;
    }

    // Obtener el camino más ancho
    int idx_camino = camino_mas_ancho(empieza_camino, acaba_camino, j);

    // Filtrar caminos inexactos
    if ((acaba_camino[idx_camino] - empieza_camino[idx_camino] < 25) || (j > 15)) parar = true;
    else parar = false;

    // Definir la orientación media
    orientacion = 0.5 * (empieza_camino[idx_camino] + 90 + acaba_camino[idx_camino] + 90);

    // Mostrar información de la lectura del laser
    // printf("\nCamino más ancho [%d, %d] -> Orientarlo hacia %f", empieza_camino[idx_camino] + 90, acaba_camino[idx_camino] + 90,  orientacion);
    // printf("\nPosibles caminos (%d): ", j);
    // for (int ii = 0; ii <= j; ii++) printf(" [%d, %d] ", empieza_camino[ii] + 90, acaba_camino[ii] + 90);
}

int
main(int argc, char **argv) {
    ros::init(argc, argv, "laser_mapping");
    ros::NodeHandle nh_, n_move;
    ros::Subscriber scan_sub;
    ros::Subscriber odom_sub;
    string laser_topic, odom_topic, vel_topic;

    nh_.param<string>("laser_topic", laser_topic, "/scan");
    nh_.param<string>("odom_topic", odom_topic, "/odom");
    cv::namedWindow("MAP", CV_WINDOW_AUTOSIZE);
    mymap = cv::Mat(MXSIZE, MYSIZE, CV_8UC3, cv::Vec3b(0, 0, 0));

    scan_sub = nh_.subscribe<sensor_msgs::LaserScan>(laser_topic, 10, &readLaser);
    odom_sub = nh_.subscribe<nav_msgs::Odometry>(odom_topic, 10, &processOdom);

    // Interfaz para acceder a parámetros privados
    ros::NodeHandle nh_vel_("~");
    nh_vel_.param<std::string>("vel_topic", vel_topic, "cmd_vel");
    cmd_vel_pub = n_move.advertise<geometry_msgs::Twist>(vel_topic, 1, true);

    ros::Rate r(10);
    while (ros::ok()) {
        check();
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}


