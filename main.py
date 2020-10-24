import numpy as np
import pygame
from pygame.locals import *
import random
from PIL import Image
from Point import *
import rt
import math
import threading
from sys import exit

def raytrace():
    #Raytraces the scene progessively
    i = 0
    while i < 50:
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
    print("ended")

def getFrame():
    # grabs the current image and returns it
    pixels = np.roll(px,(1,2),(0,1))
    return pixels


#pygame stuff
h,w=550,550
border=50
pygame.init()
screen = pygame.display.set_mode((w+(2*border), h+(2*border)))
pygame.display.set_caption("2D Raytracing")
done = False
clock = pygame.time.Clock()

#init random
random.seed()

#image setup
i = Image.new("RGB", (500, 500), (0, 0, 0) )
px = np.array(i)

#reference image for background color
im_file = Image.open("fondo.png")
ref = np.array(im_file)

#light positions
sources = [ Point(195, 200), Point( 294, 200) ]

#light color
#light = np.array([1, 1, 0.75])
light = np.array([1, 1, 1])

#warning, point order affects intersection test!!
segments = [
            ([Point(180, 135), Point(215, 135)]),
            ([Point(285, 135), Point(320, 135)]),
            ([Point(320, 135), Point(320, 280)]),
            ([Point(320, 320), Point(320, 355)]),
            ([Point(320, 355), Point(215, 355)]),
            ([Point(180, 390), Point(180, 286)]),
            ([Point(180, 286), Point(140, 286)]),
            ([Point(320, 320), Point(360, 320)]),
            ([Point(180, 250), Point(180, 135)]),
            ]


#PINTA DIAGONALES
def pintarDiagonales(punto1,punto2):
    colorSegm=(151, 210, 23)
    m = (punto2.y - punto1.y)/(punto2.x - punto1.x);
    b = punto1.y - (m * punto1.x);
    
    if(int(punto1.x)<int(punto2.x)):
        i = int(punto1.x);
        end = int(punto2.x);
    else:
        end = int(punto1.x);
        i = int(punto2.x);
        
    for x in range(i,end):
        y = m*x + b;
        px[int(x)][int(y)]=colorSegm;

#Pinta la segmento entre esos 2 puntos
def pintarLinea(punto1,punto2):
    #Color de los segmentos
    colorSegm=(210, 23, 23);

    #Cuando el valor de X es el mismo y cambia su posicion en Y (Segmento Vertical)
    if punto1.x==punto2.x:
        #Verifica cual punto es el que tiene menor valor en el eje Y
        #Y cambia de color los pixeles del segmento
        if punto1.y<punto2.y:
            for i in range(punto1.y,punto2.y+1):
                px[int(punto1.x)][int(i)]=colorSegm
        else:
            for i in range(punto2.y,punto1.y+1):
                px[int(punto1.x)][int(i)]=colorSegm

    #Cuando el valor de Y es el mismo y cambia su posicion en X (Segmento Horizontal)          
    elif punto1.y==punto2.y:
        #Verifica cual punto es el que tiene menor valor en el eje X
        #Y cambia de color los pixeles del segmento
        if punto1.x<punto2.x:
            for e in range(punto1.x,punto2.x+1):
                px[int(e)][int(punto2.y)]=colorSegm
        else:
            for e in range(punto2.x,punto1.x+1):
                px[int(e)][int(punto2.y)]=colorSegm
    else:
        #Al no coincidir ninguno de los valores del par es una diagonal
        pintarDiagonales(punto1,punto2)
        
#Pinta todos los segmentos
def pintarSegmentos(segments):
    #Para cada segmento envia los 2 puntos.
    for segment in segments:
        pintarLinea(segment[0],segment[1])
        
#Pinta los segmentos para ver donde choca.
pintarSegmentos(segments)




#thread setup
#t = threading.Thread(target = raytrace) # f being the function that tells how the ball should move
#t.setDaemon(True) # Alternatively, you can use "t.daemon = True"
#t.start()

#main loop
raytrace()
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
        pygame.draw.line(surface, (255,255,255), (10,15), (50,25))
        clock.tick(60)


