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

""" ------------------------CODIGO ORIGINAL DEL PROFE--------------------
def raytrace():
    #Raytraces the scene progessively
    while True :
        #random point in the image
        point = Point(random.uniform(0, 500), random.uniform(0, 500))
        #pixel color
        pixel = 0

        for source in sources:
            #calculates direction to light source

            dir = source-point
            pintarLinea(Point(250,250),point)
            #add jitter
            #dir.x += random.uniform(0, 25)
            #dir.y += random.uniform(0, 25)

            #distance between point and light source
            length = rt.length(dir)
            #normalized distance to source
            length2 = rt.length(rt.normalize(dir))

            free = True
            for seg in segments:
                #check if ray intersects with segment
                dist = rt.raySegmentIntersect(point, dir, seg[0], seg[1])
                #if intersection, or if intersection is closer than light source
                if  dist > 0 and length2>dist:
                    free = False
                    break

            if free:
                intensity = (1-(length/500))**2
                #print(len)
                #intensity = max(0, min(intensity, 255))
                values = (ref[int(point.y)][int(point.x)])[:3]
                #combine color, light source and light color
                values = values * intensity * light

                #add all light sources
                pixel += values

            #average pixel value and assign
            px[int(point.x)][int(point.y)] = pixel // len(sources)
            i= i+1
------------------------CODIGO ORIGINAL DEL PROFE--------------------"""

#coeficiente de absorción

#--------------------------------Raytrace--------------------------------
#------------------------------------------------------------------------

def raytrace(ray, sonar):
    #Raytraces the scene progessively
    if(ray == None):
        #Obtiene la posición del sonar(por ahora está asignado al origen del rayo,
        #pero probablemente lo manejemos por aparte)
        point = Point(random.uniform(0, 550), random.uniform(0, 550));
        point = sonar.dir
        #ray = maximizeDirection(Ray(255.0,sonar.pos , point));
        ray = maximizeDirection(Ray(255.0,sonar.pos , point));
    if(ray.intensity < 1):
        return
    #pixel color
    
    pixel = 0
    source = ray.origin
    point = ray.dir
    
    dir = point-source
    #print(source);
    #print(point);

    #Ya pintar linea contiene el dibujar diagonal así se usa solo una funcion para dibujar cualquier linea
    
    pintarLinea(source,point);
    
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
        ry = generateReflectedRay(intersectionPoint, ang, ray);
        #Genera el eco
        eco = Ray(ray.intensity, intersectionPoint, sonar.pos);
        """
        En esta parte se comprueba que el eco pueda llegar a el sonar, entonces se realiza el mismo procedimiento de
        arriba donde comprobamos si toca a alguna pared. Por alguna razón que no entiendo, está roto y hace colisiones
        con segmentos que nada que ver y con los que debería colisionar no los detecta.
        """
         
        #####################################################
        #PARTE QUE FALLA
        #####################################################
        tempDist = 10000
        reachSonar = True;
        length = rt.length(eco.dir)
        length2 = rt.length(rt.normalize(eco.dir))
        for seg in segments:
        #check if ray intersects with segment
            dist = rt.raySegmentIntersect(eco.origin, eco.dir, seg[0], seg[1])
            #if intersection, or if intersection is closer than light source
            tempInterPoint= rt.intersectionPoint(eco.origin, eco.dir, dist);
            #print(int(eco.origin.x) == int(tempInterPoint.x) )
            #print(int(eco.origin.y) == int(tempInterPoint.y))
            if length2>dist and not (int(eco.origin.x) == int(tempInterPoint.x) 
                                     and int(eco.origin.y) == int(tempInterPoint.y)):
                if(dist <tempDist and dist > 0):
                    reachSonar = False
                    #intersectionPoint = rt.intersectionPoint(source, dir, dist);
                    tempDist = dist
        #####################################################
        #PARTE QUE FALLA
        #####################################################
        
        #print para conocer si atravieza o no la pared
        print(isNotCrossingTheWall(intersectionPoint, eco, ry, segment))
        #print para saber si hay alguna pared en medio
        print (reachSonar)      
        
        #Si no hay pared y no atravieza la pared
        if(reachSonar and isNotCrossingTheWall(intersectionPoint, eco, ry, segment)):
            #Se toma el eco como valido
            pintarLinea(eco.dir , eco.origin);
            print("Hola, soy un eco :3")
        
        
        #####
        #---------Pinta una cruz en el punto de intersección---------------------
        px[int(intersectionPoint.x)][int(intersectionPoint.y)] = (0,255,255);
        px[int(intersectionPoint.x+1)][int(intersectionPoint.y)] = (0,255,255);
        px[int(intersectionPoint.x-1)][int(intersectionPoint.y)] = (0,255,255);
        px[int(intersectionPoint.x)][int(intersectionPoint.y-1)] = (0,255,255);
        px[int(intersectionPoint.x)][int(intersectionPoint.y+1)] = (0,255,255);
        #-------------------------------------------------------------------------

        distance = rt.length(intersectionPoint)
        ry.intensity = getIntensityLosseByDistance(ry.intensity, distance);
        raytrace(ry, sonar);
        
        #values = (ref[int(point.y)][int(point.x)])[:3]
        #combine color, light source and light color
        #values = values * intensity * light

        #add all light sources
        #pixel += values

        #average pixel value and assign
        #px[int(point.x)][int(point.y)] = pixel // len(sources)"""
    #print("ended")



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
    
    #print(segAngle1)
    #print(segAngle2)
    #print(reflexAngle)
    #print(eco.origin);
    #print(eco.dir)
    #print(ecoAngle)    
    
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

def getIntensityLosseByDistance(intensity, distance):
    newIntensity = intensity * pow(math.e , -beta*distance);
    #print(str(newIntensity) + " " + str(distance)) 
    return newIntensity;

def generateReflectedRay(point, angle, sourceRay):
    origin = sourceRay.origin;
    #Pasa el angulo de grados a radianes
    angle = angle*(math.pi/180)
    #Obtiene el seno y el coseno del angulo 
    cosAngle = math.cos(angle);
    senAngle = math.sin(angle)
    #traslada hace una traslación hacia el origen del plano cartesiano.
    #El origen del rayo corresponde al origen del plano cartesiano con el fin de rotar el rayo a partir de ese punto
    originX = 0;
    originY = 0
    translationX = point.x
    translationY = point.y 
    y = origin.y - translationY
    x = origin.x - translationX
    #Hace el calculo de X' y Y' utilizando la formula de rotación de ejes
    xPrima = (x * cosAngle) - (y * senAngle)
    yPrima = (y * cosAngle) + (x * senAngle)
    #dir = maximizeDirection(point, Point(origin.x, int(y)))
    
    #Se devuelven ambos puntos a sus posiciónes
    originX = translationX
    originY = translationY
    xPrima += translationX
    yPrima += translationY
    intensity = sourceRay.intensity
    ray = Ray(intensity, Point(originX, originY), Point(xPrima, yPrima))
    ray = maximizeDirection(ray) 
    return ray



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


 #PINTA DIAGONALES
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
        
#Pinta todos los segmentos
def pintarSegmentos(segments):
    #Para cada segmento envia los 2 puntos.
    for segment in segments:
        pintarLinea(segment[0],segment[1])
    
#Pone el punto de le dirección en los límites del espacio
def maximizeDirection(ray):
    origin = ray.origin
    dir = ray.dir;
    if(origin.x == dir.x):
        if(origin.y< dir.y):
            dir.y = 0;
        else:
            dir.y = h;
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
"""
Mae, los segmentos estan ordenados para que sea un poco más fácil de debuguear.
El asunto aquí es que está detectando colisiones con segmentos que no deberían,
ya que ni siquiera está en la misma dirección.
Y con otros directamente no detecta la colisión cuando el eco claramente pasa por encima.
"""
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
#sonar =  Sonar(Point(190,150), Point(270,195));
sonar =  Sonar(Point(190,150), Point(100,165));
#---------------------------------------------------------------------------
#"""
t = threading.Thread(target = raytrace(None, sonar.clone())) # f being the function that tells how the ball should move
t.setDaemon(True) # Alternatively, you can use "t.daemon = True"
t.start()
#"""
#raytrace(None);
while True:
        
        for event in pygame.event.get():
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


