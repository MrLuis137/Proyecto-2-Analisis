import numpy as np
import pygame
from pygame.locals import *
import random
from PIL import Image
from Point import *
from Sonar import *
import rt
import math
import threading
from sys import exit
from Ray import *

#coeficiente de absorción

#--------------------------------Raytrace--------------------------------
#------------------------------------------------------------------------

# El angulo de escaneo corresponde al angulo en que se dirigió el primer rayo
# donde 0 corresponde a la dirección del sonar.
def raytrace(ray, sonar, depth, scanningAngle):
    #Raytraces the scene progessively
    if(ray == None):
        #Obtiene la posición del sonar(por ahora está asignado al origen del rayo,
        #pero probablemente lo manejemos por aparte)
        point = Point(random.uniform(0, 550), random.uniform(0, 550));
        point = sonar.dir
        #ray = maximizeDirection(Ray(255.0,sonar.pos , point));
        ray = maximizeDirection(Ray(255.0,sonar.pos , point));
    #¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡
    # Parametro que determina la profundidad de la recursión
    # Setear un número muy alto puede llevar a una duración excesiva
    #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    if(depth == 3 ):
        return
    
    
    source = ray.origin
    point = ray.dir
    
    dir = point-source
    #print(source);
    #print(point);

    #Ya pintar linea contiene el dibujar diagonal así se usa solo una funcion para dibujar cualquier linea
    
    #pintarLinea(source,point);
    
    #Dibuja los puntos
    #   Del punto:
    #px[int(point.x)][int(point.y)]=(255,0,255)
    #   Del la posicion del sonar:
    #px[int(source.x)][int(source.y)]=(255,0,255)
    
    #add jitter
    #dir.x += random.uniform(0, 25)
    #dir.y += random.uniform(0, 25)

    #distance between point and light source
    length = rt.length(dir)
    #normalized distance to source
    length2 = rt.length(rt.normalize(dir))

    free = True
    intersectionPoint = Point(0,0);
    tempDist = 10000
    segment = 0        
    for seg in segments:
        #check if ray intersects with segment
        dist = rt.raySegmentIntersect(source, dir, seg[0], seg[1])
        #if intersection, or if intersection is closer than light source
        tempInterPoint= rt.intersectionPoint(source, dir, dist);
        if length2>dist and not (int(source.x) == int(tempInterPoint.x) 
                                 and int(source.y) == int(tempInterPoint.y)):
            if(dist <tempDist and dist > 0):
                free = False
                intersectionPoint = rt.intersectionPoint(source, dir, dist);
                tempDist = dist
                segment = seg
                
    
    if not free:
        ##### Prueba para generar la reflexión
        #print(intersectionPoint)
        ang=getAngle(source,intersectionPoint,segment)
        ray.traveledDistance += calculateDistance(ray.origin, intersectionPoint)
        #print(calculateDistance(ray.origin, intersectionPoint))
        ry = generateReflectedRay(intersectionPoint, ang, ray);
        ry.traveledDistance += calculateDistance(ray.origin, intersectionPoint)
        #Genera el eco
        eco = Ray(ray.intensity, intersectionPoint, sonar.pos);
        eco.traveledDistance = ry.traveledDistance

        tempDist = 10000
        reachSonar = True;
        ecodir=eco.dir-eco.origin
        
        length = rt.length(ecodir)
        length2 = rt.length(rt.normalize(ecodir))
        for seg in segments:
        #check if ray intersects with segment
            dist = rt.raySegmentIntersect(eco.origin, ecodir, seg[0], seg[1])
            #if intersection, or if intersection is closer than light source
            tempInterPoint= rt.intersectionPoint(eco.origin, ecodir, dist);
            if length2>dist and not (int(eco.origin.x) == int(tempInterPoint.x) 
                                     and int(eco.origin.y) == int(tempInterPoint.y)):
                if(dist <tempDist and dist > 0):
                    reachSonar = False
                    #intersectionPoint = rt.intersectionPoint(source, dir, dist);
                    tempDist = dist
                    
        #Si no hay pared y no atravieza la pared
        if(reachSonar and isNotCrossingTheWall(intersectionPoint, eco, ry, segment)):
            #obtiene la distancia recorrida total
            dist = (ray.traveledDistance + calculateDistance(eco.origin, eco.dir))
            #Obtioene la perdida de intensidad
            intensity = getIntensityLosseByDistance(eco.intensity, dist)
            pt= Point(sonar.pos.x + (dist /2), sonar.pos.y);
            translationX = sonar.pos.x
            translationY = sonar.pos.y
            #Obtiene el angulo al que está viendo el sonar y le suma el angulo al cual fue
            #dirigido el rayo original
            angle = getAngleOfPoint(sonar.dir.x - translationX, sonar.dir.y - translationY) + scanningAngle
            #se hace la rotación correspondiente
            pt = rotatePoint(sonar.pos, pt,angle)
            #si el punto a pintar no se sale de la escena
            if(pt.x > 0 and pt.x < w and pt.y > 0 and pt.y < h):
                #pinta el punto
                pintarPoint(int(pt.x),int(pt.y),(intensity,intensity,intensity))
            #Se toma el eco como valido
            
            #DESCOMENTAR PARA VER ECOS
            #pintarLinea(eco.dir , eco.origin); 
        
        
        #- pintarLinea(source , ry.origin);
        
        for aux in getAnglesSec(ang,4):
            ryS = generateReflectedRay(intersectionPoint, aux, ray)
            ryS.traveledDistance = ray.traveledDistance
            raytrace(ryS, sonar,depth +1, scanningAngle )
            #print("Rota:",aux,"Sale:",(90-aux/2)+aux,"Diferencia:",ang-aux)#Prubeas para ver el comportamiento de los ang secundarios
            
            #pintarLinea(ryS.dir , ryS.origin)            
            
        px[int(sonar.pos.x)][int(sonar.pos.y)] = (0,255,255);
        #return#Descomentar para solo ver 1 rayo y sus secundarios
        #####
        #---------Pinta una cruz en el punto de intersección---------------------
        #px[int(intersectionPoint.x)][int(intersectionPoint.y)] = (0,255,255);
        #px[int(intersectionPoint.x+1)][int(intersectionPoint.y)] = (0,255,255);
        #px[int(intersectionPoint.x-1)][int(intersectionPoint.y)] = (0,255,255);
        #px[int(intersectionPoint.x)][int(intersectionPoint.y-1)] = (0,255,255);
        #px[int(intersectionPoint.x)][int(intersectionPoint.y+1)] = (0,255,255);
        #-------------------------------------------------------------------------

        ry.traveledDistance = ray.traveledDistance
        raytrace(ry, sonar, depth + 1, scanningAngle);
        



#----------------------------Fin Raytrace--------------------------------
#------------------------------------------------------------------------


#------------------------Crossing the wall-------------------------------
#------------------------------------------------------------------------
def isNotCrossingTheWall(intersectionPoint, eco, ray, segment):
    #pintarLinea(eco.dir , eco.origin);
    
    #Obtiene cuanto hay que trasladar los puntos para que la intersección
    #coresponda a 0,0
    translationX = intersectionPoint.x
    translationY = intersectionPoint.y 
    
    #Obtiene el angulo de uno de los puntos del segmento con respecto
    #a la intersección
    y = segment[0].y - translationY
    x = segment[0].x - translationX
    segAngle1  = getAngleOfPoint(x, y)
    #AngleSeg2
    
    #Obtiene el angulo de uno de los puntos del segmento con respecto
    #a la intersección
    y = segment[1].y - translationY
    x = segment[1].x - translationX
    segAngle2  = getAngleOfPoint(x, y)
    
    #Se asegura que el angulo mayor sea el del segAngle2
    if(segAngle1 > segAngle2):
        temp = segAngle1
        segAngle1 = segAngle2
        segAngle2 = temp
        
    
    #Obtiene el angulo de el reflejo con respecto a la intersección
    y = ray.dir.y - translationY
    x = ray.dir.y - translationX
    reflexAngle  = getAngleOfPoint(x, y)

    #Obtiene el angulo de el eco con respecto a la intersección
    y = eco.dir.y - translationY
    x = eco.dir.x - translationX
    ecoAngle  = getAngleOfPoint(x, y)
    
    #Si el angulo del reflejo es mayor que segAngle2
    #y el eco es mayor que el angulo segAngle2
    if(reflexAngle > segAngle2):
        if(ecoAngle > segAngle2):
            return True;
        
    #Si el angulo del reflejo es menor que segAngle2
    #y el eco es menor que el angulo segAngle2
    if(reflexAngle < segAngle2):
        if(ecoAngle < segAngle2):
            return True;
    
    #Casos especiales
    if(reflexAngle > segAngle2 and reflexAngle > segAngle1):
         if(ecoAngle < segAngle2 or ecoAngle > segAngle1):
            return True;
    
    if(reflexAngle < segAngle2 and reflexAngle < segAngle1):
         if(ecoAngle > segAngle2 or ecoAngle < segAngle1):
            return True; 
    return False
    
#---------------Obtener el angulo de un punto----------------------------
#------------------------------------------------------------------------

#Obtiene el angulo de un punto con respecto a 0 x,0 y 
def getAngleOfPoint(x,y):
    #print(x,y)
    a = 0
    if(x > 0 and y > 0):
        a = math.atan(y/x)
    elif(x < 0 and y > 0):
        a = math.atan(y/x) + math.pi
    elif(x < 0 and y < 0):
        a = math.atan(y/x) + math.pi
    elif (x > 0 and y < 0):
        a = math.atan(y/x) + (2 * math.pi)
    elif(x == 0 and y > 0):
        a = math.pi/2
    elif(x == 0 and y < 0):
        a = 2 * math.pi
    elif(y == 0 and x > 0):
        a = 0
    elif(y == 0 and x < 0):
        a = math.pi
    #print(a)
    return a * (180/ math.pi);

#-----------------------Fin Crossing the wall----------------------------
#------------------------------------------------------------------------



def getFrame():
    # grabs the current image and returns it
    pixels = np.roll(px,(1,2),(0,1))
    return pixels


#---------------------------Perdida de energía---------------------------
#------------------------------------------------------------------------
def getIntensityLosseByDistance(intensity, distance):
    newIntensity = intensity * pow(math.e , -beta*distance);
    #print(str(newIntensity) + " " + str(distance)) 
    return newIntensity;

#--------------------------Calcular Distancia----------------------
#------------------------------------------------------------------------

def calculateDistance(point1, point2):
    c1 = abs(point1.x - point2.x)
    c2 = abs(point1.y - point2.y)
    return math.sqrt(c1**2 + c2**2) 

#---------------------------Rayo reflejado---------------------------
#------------------------------------------------------------------------
def generateReflectedRay(point, angle, sourceRay):
    origin = sourceRay.origin;

    intensity = sourceRay.intensity
    #ray = Ray(intensity, Point(originX, originY), Point(xPrima, yPrima))
    ray = Ray(intensity, Point(point.x, point.y), rotatePoint(point, origin , angle))
    ray = maximizeDirection(ray) 
    return ray


#---------------------------Rotación del punto---------------------------
#------------------------------------------------------------------------

def rotatePoint(axisPoint, point, angle ):
    #Pasa el angulo de grados a radianes
    translationX = axisPoint.x
    translationY = axisPoint.y 
    y = point.y - translationY
    x = point.x - translationX
    angle = angle*(math.pi/180)
    cosAngle = math.cos(angle);
    senAngle = math.sin(angle)
    xPrima = (x * cosAngle) - (y * senAngle)
    yPrima = (y * cosAngle) + (x * senAngle)
    xPrima += translationX
    yPrima += translationY
    return Point(xPrima, yPrima);

#---------------------------getAngleSec---------------------------
#------------------------------------------------------------------------

#Recibe el angulo de rotacion y la cantidad de segmentos secundarios que va a generar
def getAnglesSec(angR,cant):
    angulos=[]#angulos
    ang=90-(angR/2)#Obtine el angulo en el que llega 
    #Define los grandos +- que va a variar el los secundarios
    grados=30
    #Determina los rangos
    rang1=grados
    rang2=-grados
    #calcula las diferencis o bordes
    dif1=(ang+angR+rang1)
    dif2=(ang+angR+rang2)
    #Pregunta para ver si se salen de 180
    if (dif1)>180:
        rang1-=(dif1-180)
        rang2+=(dif1-180)
        #print("si 1")#Prueba si hay cambio en el primer if    
    if (dif2)>180:
        rang2-=(dif1-180)
        rang1+=(dif1-180)
        #print("si 2")#Prueba si hay cambio en el segundo if 
    #Genera los ángulos en los que van a salir los secundarios    
    while len(angulos)<cant:
        angNew=random.uniform(angR+rang1,angR+rang2)
        angulos.append(angNew)        
    return angulos


def segVertical(origen,destino,seg):
    punto=Point(seg[0].x,origen.y)
    hip=rt.length(destino-origen)
    ca=rt.length(destino-punto)
    if origen.y!=destino.y:
        return math.acos(ca/hip)*180/math.pi
    else:#Está en angulo de noventa.
        return 90

def segHorizontal(origen,destino,seg):
    punto=Point(origen.x,seg[0].y)
    hip=rt.length(destino-origen)
    ca=rt.length(destino-punto)
    if origen.x!=destino.x:
        return math.acos(ca/hip)*180/math.pi   
    else:#Está en angulo de noventa.
        return 90
    
def segDiagonal(origen,destino,seg):
    if rt.length(seg[0]-origen)<rt.length(seg[1]-origen):
        punto=seg[0]
    else:
        punto=seg[1]
        
    part1=origen.y*(destino.x-punto.x)+destino.y*(punto.x-origen.x)+punto.y*(origen.x-destino.x)    
    part2=(origen.x-destino.x)*(destino.x-punto.x)+(origen.y-destino.y)*(destino.y-punto.y)
    ang=math.atan(part1/part2) 
    ang=ang*180/math.pi
    return ang

#---------------------------get Angle---------------------------------
#------------------------------------------------------------------------
    
def getAngle(origen,destino,seg):#Punto de donde sale el rayo, punto donde interseca y el segmento con el que choca
    #Decidir si el segmentoes verical
    if seg[0].x==seg[1].x:
        ang=segVertical(origen,destino,seg)
        ang=(90-ang)*2
        if seg[0].x<origen.x:#Cuando el segmento está del lado izquierdo
            if origen.y<destino.y:
                return ang   
            else:
                return -ang    
        else:#Cuando el segmento está del lado izquierdo
            if origen.y<destino.y:
                return -ang   
            else:
                return ang           
    #Horizonatal
    elif seg[0].y==seg[1].y:
        ang=segHorizontal(origen,destino,seg)
        ang=(90-ang)*2     
        if seg[0].y<origen.y:#Cuando el segmento está arriba
            if origen.x<destino.x:
                return -ang   
            else:
                return ang    
        else:#Cuando el segmento está del lado izquierdo
            if origen.x<destino.x:
                return ang   
            else:
                return -ang
    else:
        ang=segDiagonal(origen,destino,seg)
        ang=(90-ang)*2
        return ang

#-------------------------------Pintar Punto----------------------------
#-----------------------------------------------------------------------
def pintarPoint(x,y,color):
    colorCruz=colorArist=list(color)
    #Se asegura de que la resta sea valida sino el color secundario es 0
    if color[0]-20<0:
        colorCruz[0]=colorCruz[1]=colorCruz[2]=0
    else:
        colorCruz[0]=colorCruz[1]=colorCruz[2]=color[0]-20     
    if color[0]-40<0:
        colorArist[0]=colorArist[1]=colorArist[2]=0
    else:
        colorArist[0]=colorArist[1]=colorArist[2]=color[0]-40

    colorCruz=tuple(colorCruz)
    colorArist=tuple(colorArist)    
    #Pinta el pixel central
    px[x][y] = color
    #Los 4 if principales es para preguntar por la "cruz" a partir del pixel central
    #Los if anidados es para las aristas
    if x+1<h:    
        px[x+1][y] = colorCruz
        if y-1>0:
            px[x+1][y-1] = colorArist
        if y+1<h:
            px[x+1][y+1] = colorArist
    if x-1>0:
        px[x-1][y] = colorCruz
        if y-1>0:
            px[x-1][y-1] = colorArist
        if y+1<h:
            px[x-1][y+1] = colorArist       
    #Despues de hacer esos 2 ya se revisaron las 4 aristas
    if y-1>0:
        px[x][y-1] = colorCruz   
    if y+1<h:
        px[x][y+1] = colorCruz
    
    
#---------------------------Pintar Diagonales---------------------------
#------------------------------------------------------------------------
def pintarDiagonales(punto1,punto2):
    colorSegm=(151, 210, 23)
    m = (punto2.y - punto1.y)/(punto2.x - punto1.x);
    #calcula el b
    b = punto1.y - (m * punto1.x);
    
    if(int(punto1.x)<int(punto2.x)):
        i = int(punto1.x);
        end = int(punto2.x);
    else:
        end = int(punto1.x);
        i = int(punto2.x);
    #Solo es necesario para las dibujar los rayos. Se puede borrar una vez no se necesite
    if(i<0):
        i=0
    #####
    for x in range(i,end):
        y = m*x + b;
        #Solo es necesario para las dibujar los rayos. Se puede borrar una vez no se necesite
        if(y < 0 or y >549 or x > 548):
            continue
        #####
        px[int(x)][int(y)]=colorSegm;
        x+=0.05;


#---------------------------Pintar Linea---------------------------------
#------------------------------------------------------------------------
#Pinta la segmento entre esos 2 puntos
def pintarLinea(punto1,punto2):
    #Color de los segmentos
    
    colorSegm=(210, 23, 23);

    #Cuando el valor de X es el mismo y cambia su posicion en Y (Segmento Vertical)
    if punto1.x==punto2.x:
        #Verifica cual punto es el que tiene menor valor en el eje Y
        #Y cambia de color los pixeles del segmento
        if punto1.y<punto2.y:
            for i in range(int(punto1.y),int(punto2.y+1)):
                px[int(punto1.x)][int(i)]=colorSegm
                #Solo es necesario para las dibujar los rayos. Se puede borrar una vez no se necesite
                if(i == 549):
                    break
                #####
        else:
            for i in range(int(punto2.y),int(punto1.y+1)):
                px[int(punto1.x)][int(i)]=colorSegm
                #Solo es necesario para las dibujar los rayos. Se puede borrar una vez no se necesite
                if(i == 549):
                    break
                #####

    #Cuando el valor de Y es el mismo y cambia su posicion en X (Segmento Horizontal)          
    elif punto1.y==punto2.y:
        #Verifica cual punto es el que tiene menor valor en el eje X
        #Y cambia de color los pixeles del segmento
        if punto1.x<punto2.x:
            for e in range(int(punto1.x), int(punto2.x+1)):
                px[int(e)][int(punto2.y)]=colorSegm
        else:
            for e in range(int(punto2.x), int(punto1.x+1)):
                px[int(e)][int(punto2.y)]=colorSegm
    else:
        #Al no coincidir ninguno de los valores del par es una diagonal
        pintarDiagonales(punto1,punto2)



#---------------------------Pintar segmentos-----------------------------
#------------------------------------------------------------------------
        
#Pinta todos los segmentos
def pintarSegmentos(segments):
    #Para cada segmento envia los 2 puntos.
    for segment in segments:
        pintarLinea(segment[0],segment[1])
        



#---------------------------Maximizar el rayo---------------------------
#------------------------------------------------------------------------
    
#Pone el punto de le dirección en los límites del espacio
def maximizeDirection(ray):
    origin = ray.origin
    dir = ray.dir;
    if(origin.x == dir.x):
        if(origin.y< dir.y):
            dir.y = h;
        else:
            dir.y = 0;
    else:
        m = (origin.y - dir.y)/(origin.x - dir.x);
        #calcula el b
        b = origin.y - (m * origin.x);
        if(origin.x < dir.x):
            dir.y= m*w-1 + b;
            dir.x = w-1;
            ray.dir = dir
        else:
            dir.y = m*0 + b;
            dir.x = 0;
            ray.dir = dir
    return ray;
#---------------------------Maximizar el rayo---------------------------
#------------------------------------------------------------------------

#pygame stuff
h,w=550,550
border=0
pygame.init()
screen = pygame.display.set_mode((w+(2*border), h+(2*border)))
pygame.display.set_caption("2D Raytracing")
done = False
clock = pygame.time.Clock()

#init random
random.seed()

#image setup
i = Image.new("RGB", (550, 550), (0, 0, 0) )
px = np.array(i)

#reference image for background color
#im_file = Image.open("fondo.png")
#ref = np.array(im_file)

#light positions
#sources = [ Point(195, 200), Point( 294, 200) ]

#light color
#light = np.array([1, 1, 0.75])
#light = np.array([1, 1, 1])

#warning, point order affects intersection test!!

segments = [
            #Segmento inclinado
            ([Point(150,250), Point(180, 135)]),
             #Segmento problemático 
            ([Point(320, 135), Point(320, 280)]),
            
            ([Point(160,250), Point(210, 250)]),
            ([Point(180, 135), Point(215, 135)]),
            ([Point(285, 135), Point(320, 135)]),

            ([Point(320, 320), Point(320, 355)]),
            ([Point(320, 355), Point(215, 355)]),
            ([Point(180, 390), Point(180, 286)]),
            ([Point(180, 286), Point(140, 286)]),
            ([Point(320, 320), Point(360, 320)]),
            
            ]
        
#Pinta los segmentos para ver donde choca.
pintarSegmentos(segments)


#main loop

#coeficiente de absorcion
beta = 0.00137
#----------------------------Se crea el sonar--------------------------------
#sonar =  Sonar(Point(190,150), Point(190,250));
sonar =  Sonar(Point(190,150), Point(200,165));
#---------------------------------------------------------------------------
#"""
t = threading.Thread(target = raytrace(None, sonar.clone(),0 ,0)) # f being the function that tells how the ball should move
t.setDaemon(True) # Alternatively, you can use "t.daemon = True"
t.start()
#"""
#raytrace(None);
while True:
    for event in pygame.event.get():
        mouseClick=pygame.mouse.get_pressed()
        if(mouseClick[0]==1):
            #Resetea el mapa
            px = np.array(i)
            pintarSegmentos(segments)
            posX, posY = pygame.mouse.get_pos()
            sonar.pos=Point(posX,posY)
            #print(sonar.dir)
            raytrace(None, sonar.clone(),0 ,0)
            
        if event.type == pygame.QUIT:
            running = False
            pygame.quit()
            exit()
            break
    # Clear screen to white before drawing
    screen.fill((255, 255, 255))
    # Get a numpy array to display from the simulation
    npimage=getFrame()
    # Convert to a surface and splat onto screen offset by border width and height
    surface = pygame.surfarray.make_surface(npimage)
    screen.blit(surface, (border, border))    
    pygame.display.flip()
    clock.tick(60)


