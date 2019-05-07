# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

from math import *
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

##Représentation du mur

source1=open("Voie2.txt", "r")
xPrises=[]
yPrises=[]
taillePrises=[]
instabilitésPrises=[]

for line in source1:
    #récupération de l'abscisse
    i=0
    x=''
    y=''
    q=''
    e=''
    while (line[i]!=' '):
        x+=(line[i])
        i+=1
   #récupération de l'ordonnée
    i+=1
    while (line[i]!=' '):
        y+=(line[i])
        i+=1
    #récupération de la taille
    i+=1
    while (line[i]!=' '):
        q+=(line[i])
        i+=1
    #récupération des instabilités
    i+=1
    while (line[i]!=' '):
        e+=(line[i])
        i+=1
    xPrises.append(float(x))
    yPrises.append(float(y))
    taillePrises.append(float(q))
    instabilitésPrises.append(float(e))
    
#print(yPrises[2])

#représentation des prises
#plt.axis([-1, 6, -1, 10])
#plt.plot(xPrises, yPrises, color='r', linestyle='',
#marker='o')
#plt.show()




##représentation du personnage

source2=open("Moi.txt", "r")
i=0
Lb=''
Lj=''
Lt=''
angle=''
p=''
t=''
a=source2.readline()
#lecture de la longueur des bras
while (a[i]!=' '):
    Lb+=(a[i])
    i+=1
Lbras=float(Lb)
#lecture de la longueur des jambes
i+=1
while (a[i]!=' '):
    Lj+=(a[i])
    i+=1
Ljambe=float(Lj)
#lecture de la longueur du torse
i+=1
while (a[i]!=' '):
    Lt+=(a[i])
    i+=1
Ltorse=float(Lt)
#lecture de l'angle max entre les deux jambes
i+=1
while (a[i]!=' '):
    angle+=(a[i])
    i+=1
angleSouplesse=float(angle)
#lecture du poids
i+=1
while (a[i]!=' '):
    p+=(a[i])
    i+=1
poids=float(p)
#lecture de la taille
i+=1
while (a[i]!=' '):
    t+=(a[i])
    i+=1
taille=float(t)

#on veut maintenant positionner les segments du tronc et des membres
#idée1 : on veut garder le tronc le plus droit possible
#idée2 : on essaie d'avoir le Cg plutôt au dessus de la jambe inférieure
#idée3: le tronc commence plus haut que les prises de pied
#idée4 : on préfère avoir la jambe du bas tendue en dalle

##Méthode: 1) trouver l'éq. de position du bassin (dichotomie ou mieux): c'est une courbe; 
#2)Trouver les éq. des demis-cercles d'intersection pour les bras->déf. un ensemble
#3)Relier le bassin à son projeté sur l'ensemble du haut.
#4)Calculer la longueur du torse et le coeff directeur.
#5)Si c'est supérieur à la vraie longueur: impossible.
#6)Sinon: soit le coeff. dir est positif et alors on le fait tendre vers l'infini et autrement vers -l'infini.
#7)Obj.: avoir le torse le plus droit possible => choix de tel ou tel arc de cercle à suivre

def distance(x10,y10,x20,y20):
    return ((x20-x10)**2+(y20-y10)**2)**0.5;

def minimum(a,b):
    if (a<b):
        return a
    else:
        return b
def maximum(a,b):
    if (a<b):
        return b
    else:
        return a

#on cherche le premier arc de cercle permettant de garder tendue la jambe du bas
x1=xPrises[0]
y1=yPrises[0]
x2=xPrises[1]
y2=yPrises[1]
x3=xPrises[2]
y3=yPrises[2]
x4=xPrises[3]
y4=yPrises[3]
e1=instabilitésPrises[0]
e2=instabilitésPrises[1]
e3=instabilitésPrises[2]
e4=instabilitésPrises[3]
t1=taillePrises[0]
t2=taillePrises[1]
t3=taillePrises[2]
t4=taillePrises[3]

plt.axis([-2, 8, -1, 9])
plt.plot(xPrises, yPrises, color='k', linestyle='', marker='o')
plt.show()

#while (distance(x4,y4,xPrises[-1],yPrises[-1])!=0):
dernierePrise=4
pairOuImpair=0
for nbMouvements in range(11):

    dPied=distance(x1,y1,x2,y2)
#    print ("dPied=",dPied)
    if (dPied>2*Ljambe):
        print (dPied,"blocagePied")
    elif (dPied==0):
        xIntersectionBasse=x1-Ljambe
        yIntersectionBasse=y1
        xIntersectionHaute=x1+Ljambe
        yIntersectionHaute=y1
    else:
        x12=(x1+x2)/2
        y12=(y1+y2)/2
        if (x2<x1):
            coeffDirPied=(y1-y2)/(x1-x2)
            distanceInterIntersections=2*(Ljambe**2-(dPied/2)**2)**0.5
            xIntersectionBasse=x12-(((distanceInterIntersections/2)**2)/((-1/coeffDirPied)**2+1))**0.5
            yIntersectionBasse=y12-(-1/coeffDirPied)*(-xIntersectionBasse+x12)
            xIntersectionHaute=(((distanceInterIntersections/2)**2)/((1/coeffDirPied)**2+1))**0.5+x12
            yIntersectionHaute=(-1/coeffDirPied)*(xIntersectionHaute-x12)+y12
        elif (x2>x1):
            coeffDirPied=(y1-y2)/(x1-x2)
            distanceInterIntersections=2*(Ljambe**2-(dPied/2)**2)**0.5
            xIntersectionBasse=x12+(((distanceInterIntersections/2)**2)/((-1/coeffDirPied)**2+1))**0.5
            yIntersectionBasse=y12+(-1/coeffDirPied)*(xIntersectionBasse-x12)
            xIntersectionHaute=x12-(((distanceInterIntersections/2)**2)/((1/coeffDirPied)**2+1))**0.5
            yIntersectionHaute=(-1/coeffDirPied)*(xIntersectionHaute-x12)+y12
        else:
            xIntersectionBasse=x1
            xIntersectionHaute=x1
            yIntersectionBasse=maximum(y1,y2)-Ljambe
            yIntersectionHaute=minimum(y1,y2)+Ljambe
    
    #Pour les 2e arcs de cercle, il faut compter que la tête peut monter au-dessus des mains
    
    dBras=distance(x3,y3,x4,y4) 
    
    if (dBras>2*Lbras):
        print (dBras, "blocagebras")
    elif (dBras==0):
        xIntersectionBasseBras=x3
        yIntersectionBasseBras=y3-Lbras
        xIntersectionHauteBras=x3
        yIntersectionHauteBras=y3+Lbras
    else:
        x34=(x3+x4)/2
        y34=(y3+y4)/2
        if (x4>x3):
            coeffDirBras=(y4-y3)/(x4-x3)
            distanceInterIntersectionsBras=2*(Lbras**2-(dBras/2)**2)**0.5
            xIntersectionBasseBras=x34+(((distanceInterIntersectionsBras/2)**2)/((-1/coeffDirBras)**2+1))**0.5
            yIntersectionBasseBras=y34+(-1/coeffDirBras)*(xIntersectionBasseBras-x34)
            xIntersectionHauteBras=x34-(((distanceInterIntersectionsBras/2)**2)/((-1/coeffDirBras)**2+1))**0.5
            yIntersectionHauteBras=y34-(-1/coeffDirBras)*(xIntersectionBasseBras-x34)
        elif (x4<x3):
            coeffDirBras=(y4-y3)/(x4-x3)
            distanceInterIntersectionsBras=2*(Lbras**2-(dBras/2)**2)**0.5
            xIntersectionBasseBras=x34-(((distanceInterIntersectionsBras/2)**2)/((-1/coeffDirBras)**2+1))**0.5
            yIntersectionBasseBras=y34-(-1/coeffDirBras)*(x34-xIntersectionBasseBras)
            xIntersectionHauteBras=x34+(((distanceInterIntersectionsBras/2)**2)/((-1/coeffDirBras)**2+1))**0.5       
            yIntersectionHauteBras=y34+(-1/coeffDirBras)*(x34-xIntersectionBasseBras)
        else:
            xIntersectionBasseBras=x3
            xIntersectionHauteBras=x3
            yIntersectionBasseBras=maximum(y3,y4)-Lbras
            yIntersectionHauteBras=minimum(y3,y4)+Lbras
    
    
    
    
    
    #On arrange le coefficient directeur du tronc pour placer le bassin du bon côté
    def coeffDirTronc(x10,y10,x20,y20):
        if (x20!=x10):
            return (y20-y10)/(x20-x10)
        else:
            return 10**32
    def P(x,y):
        return [x,y]
    def milieuDeLArc(P1,P2,centre):
        xMilieu=(P1[0]+P2[0])/2
        yMilieu=(P1[1]+P2[1])/2
        distMilieuCentre=distance(xMilieu,yMilieu,centre[0],centre[1])
        a=centre[0]+(Ljambe/distMilieuCentre)*(xMilieu-centre[0])
        b=centre[1]+(Ljambe/distMilieuCentre)*(yMilieu-centre[1])
        return P(a,b)
    def distPoint(P1,P2):
        return distance(P1[0],P1[1],P2[0],P2[1])
    p1=[x1,y1]
    p2=[x2,y2]
    p3=[x3,y3]
    p4=[x4,y4]
#    print("distPoint=",distPoint(p1,milieuDeLArc(P(xIntersectionBasse,yIntersectionBasse),P(xIntersectionHaute,yIntersectionHaute),p1)))
#    print("distPointp1XInter=",distPoint(P(xIntersectionBasse,yIntersectionBasse),p1))
    xTete=xIntersectionBasseBras
    yTete=yIntersectionBasseBras
    PTete=P(xTete,yTete)
#    if (distance(xTete,yTete,xIntersectionBasse,yIntersectionBasse)>=distance(xTete,yTete,xIntersectionHaute,yIntersectionHaute)):
#        xBassin=xIntersectionHaute
#        yBassin=yIntersectionHaute
#    else:
#        xBassin=xIntersectionBasse
#        yBassin=yIntersectionBasse
    for nbOptimisations in range (10):
        #optimisation du bassin
        listeBassin=[P(xIntersectionBasse,yIntersectionBasse),P(xIntersectionHaute,yIntersectionHaute)]
        for pointsBassinPotentiel in range (8):
            nouvelleListeBassin=[]
            demilongbass=int(len(listeBassin)/2)
            for longueur1 in range (demilongbass):
                nouvelleListeBassin.append(listeBassin[longueur1])
                nouvelleListeBassin.append(milieuDeLArc(listeBassin[longueur1],listeBassin[longueur1+1],p1))
            for longueur2 in range (int(demilongbass-1)):
                nouvelleListeBassin.append(listeBassin[longueur2+demilongbass])
                nouvelleListeBassin.append(milieuDeLArc(listeBassin[longueur2+demilongbass],listeBassin[longueur2+1+demilongbass],p2))
            nouvelleListeBassin.append(listeBassin[2*demilongbass-1])
            nouvelleListeBassin.append(milieuDeLArc(listeBassin[-1],listeBassin[2*demilongbass-1],p2))
            listeBassin=nouvelleListeBassin
        PBassin=listeBassin[0]
        for indiceBassin in range (len(listeBassin)):
            if (distPoint(PBassin,PTete)>distPoint(listeBassin[indiceBassin],PTete)):
                PBassin=listeBassin[indiceBassin]
        xBassin=PBassin[0]
        yBassin=PBassin[1]
        #optimisation de la tête
        listeTete=[P(xIntersectionBasseBras,yIntersectionBasseBras),P(xIntersectionHauteBras,yIntersectionHauteBras)]
        for pointsTetePotentiels in range (8):
            nouvelleListeTete=[]
            demilongtete=int(len(listeTete)/2)
            for longueur1 in range (demilongtete):
                nouvelleListeTete.append(listeTete[longueur1])
                nouvelleListeTete.append(milieuDeLArc(listeTete[longueur1],listeTete[longueur1+1],p3))
            for longueur2 in range (int(demilongtete-1)):
                nouvelleListeTete.append(listeTete[longueur2+demilongtete])
                nouvelleListeTete.append(milieuDeLArc(listeTete[longueur2+demilongtete],listeTete[longueur2+1+demilongtete],p4))
            nouvelleListeTete.append(listeTete[2*demilongtete-1])
            nouvelleListeTete.append(milieuDeLArc(listeTete[-1],listeTete[2*demilongtete-1],p4))
            listeTete=nouvelleListeTete
        PTete=listeTete[0]
        for indiceTete in range (len(listeTete)):
            if (distPoint(PTete,PBassin)>distPoint(listeTete[indiceTete],PBassin)):
                PTete=listeTete[indiceTete]
        xTete=PTete[0]
        yTete=PTete[1]
    
#    if (abs(coeffDirTronc(xTete,yTete,xIntersectionBasse,yIntersectionBasse))>abs(coeffDirTronc(xTete,yTete,xIntersectionHaute,yIntersectionHaute))):
#        xBassin=xIntersectionBasse
#        yBassin=yIntersectionBasse
#    else:
#        xBassin=xIntersectionHaute
#        yBassin=yIntersectionHaute
        
    #On arrange la taille du tronc/torse (marche sans BlocageTorse)
    LtorseCourant=distance(xBassin,yBassin,xTete,yTete)
#    print("LtorseCourant=",LtorseCourant)
    if (LtorseCourant>Ltorse):
        print("BlocageTorse")
        
    else:
    #si coeffDirTronc>0, on rapproche la tête du bras le plus à gauche et sinon l'inverse 
    #    if (coeffDirTronc(xTete,yTete,xBassin,yBassin)>0):
    #        for k in range (20):
    #            xMilieu=(xTete+xIntersectionHauteBras)/2
    #            yMilieu=(yTete+yIntersectionHauteBras)/2
    #            distMilieuPoint4=distance(xMilieu,yMilieu,x4,y4)
    #            xMilieuCercle=(Lbras/distMilieuPoint4)*xMilieu-((Lbras-distMilieuPoint4)/distMilieuPoint4)*x4
    #            yMilieuCercle=(Lbras/distMilieuPoint4)*yMilieu-((Lbras-distMilieuPoint4)/distMilieuPoint4)*y4
    #            if (distance(xBassin,yBassin,xMilieuCercle,yMilieuCercle)<=Ltorse):
    #                xTete=xMilieuCercle
    #                yTete=yMilieuCercle
    #            else:
    #                xIntersectionHauteBras=xMilieuCercle
    #                yIntersectionHauteBras=yMilieuCercle
    #                
    #    else:
    #        for k in range (15):
    #            xMilieu=(xTete+xIntersectionHauteBras)/2
    #            yMilieu=(yTete+yIntersectionHauteBras)/2
    #            distMilieuPoint3=distance(xMilieu,yMilieu,x3,y3)
    #            xMilieuCercle=(Lbras/distMilieuPoint3)*xMilieu-((Lbras-distMilieuPoint3)/distMilieuPoint3)*x3
    #            yMilieuCercle=(Lbras/distMilieuPoint3)*yMilieu-((Lbras-distMilieuPoint3)/distMilieuPoint3)*y3
    #            if (distance(xBassin,yBassin,xMilieuCercle,yMilieuCercle)<=Ltorse):
    #                xTete=xMilieuCercle
    #                yTete=yMilieuCercle
    #            else:
    #                xIntersectionHauteBras=xMilieuCercle
    #                yIntersectionHauteBras=yMilieuCercle
        xGaucheLarge=xBassin+(Ltorse/distance(xBassin,yBassin,x3,y3))*(x3-xBassin)
        yGaucheLarge=yBassin+(Ltorse/distance(xBassin,yBassin,x3,y3))*(y3-yBassin)
        xDroiteLarge=xBassin+(Ltorse/distance(xBassin,yBassin,x4,y4))*(x4-xBassin)
        yDroiteLarge=yBassin+(Ltorse/distance(xBassin,yBassin,x4,y4))*(y4-yBassin)
        #print("distance bassin_xGaucheLarge=",distance(xGaucheLarge,yGaucheLarge,xBassin,yBassin))
        #print("distance bassin_xDroiteLarge=",distance(xDroiteLarge,yDroiteLarge,xBassin,yBassin))
#        print("distance GaucheLArge_DroiteLArge=",distance(xGaucheLarge,yGaucheLarge,xDroiteLarge,yDroiteLarge))
#        print("distance xGaucheLarge_x4=",distance(xGaucheLarge,yGaucheLarge,x4,y4))
#        print("distance xDroiteLarge_x3=",distance(xDroiteLarge,yDroiteLarge,x3,y3))
        if (distance(xGaucheLarge,yGaucheLarge,x3,y3)>Lbras or distance(xDroiteLarge,yDroiteLarge,x4,y4)>Lbras):
            xTete=xIntersectionBasseBras
            yTete=yIntersectionBasseBras
        else:
            #Determination de xGauche
            if (distance(xGaucheLarge,yGaucheLarge,x4,y4)<=Lbras):
                xGauche=xGaucheLarge
                yGauche=yGaucheLarge
            else:
                xDInt=xDroiteLarge
                yDInt=yDroiteLarge
                for k in range (100):
                    xMilieu=(xDInt+xGaucheLarge)/2
                    yMilieu=(yDInt+yGaucheLarge)/2
                    distMilieuBassin=distance(xMilieu,yMilieu,xBassin,yBassin)
                    xMilieuCercle=(Ltorse/distMilieuBassin)*xMilieu-((Ltorse-distMilieuBassin)/distMilieuBassin)*xBassin
                    yMilieuCercle=(Ltorse/distMilieuBassin)*yMilieu-((Ltorse-distMilieuBassin)/distMilieuBassin)*yBassin
                    if (distance(xMilieuCercle,yMilieuCercle,x4,y4)>Lbras):
                        xGaucheLarge=xMilieuCercle
                        yGaucheLarge=yMilieuCercle
                    else:
                        xDInt=xMilieuCercle
                        yDInt=yMilieuCercle
                xGauche=xMilieuCercle
                yGauche=yMilieuCercle
#            print ("distance xGauche_x4=",distance(x4,y4,xGauche,yGauche))
            #Determination de xDroite
            if (distance(xDroiteLarge,yDroiteLarge,x3,y3)<=Lbras):
                xDroite=xDroiteLarge
                yDroite=yDroiteLarge
            else:
                xGInt=xGaucheLarge
                yGInt=yGaucheLarge
                for i in range (100):
                    xMilieu=(xGInt+xDroiteLarge)/2
                    yMilieu=(yGInt+yDroiteLarge)/2
                    distMilieuBassin=distance(xMilieu,yMilieu,xBassin,yBassin)
                    xMilieuCercle=(Ltorse/distMilieuBassin)*xMilieu-((Ltorse-distMilieuBassin)/distMilieuBassin)*xBassin
                    yMilieuCercle=(Ltorse/distMilieuBassin)*yMilieu-((Ltorse-distMilieuBassin)/distMilieuBassin)*yBassin
                    if (distance(xMilieuCercle,yMilieuCercle,x3,y3)>Lbras):
                        
                        xDroiteLarge=xMilieuCercle
                        yDroiteLarge=yMilieuCercle
                    else:
                        xGInt=xMilieuCercle
                        yGInt=yMilieuCercle
                xDroite=xMilieuCercle
                yDroite=yMilieuCercle
#            print ("distance xDroite_x3=",distance(x3,y3,xDroite,yDroite))
#            print ("distance xGauche_xDroite=",distance(xGauche,yGauche,xDroite,yDroite))
            #Optimisation du coeffDirTronc
            if (xGauche<xDroite):
                for j in range (100):
                    xMilieu=(xDroite+xGauche)/2
                    yMilieu=(yDroite+yGauche)/2
                    distMilieuBassin=distance(xMilieu,yMilieu,xBassin,yBassin)
                    xMilieuCercle=(Ltorse/distMilieuBassin)*xMilieu-((Ltorse-distMilieuBassin)/distMilieuBassin)*xBassin
                    yMilieuCercle=(Ltorse/distMilieuBassin)*yMilieu-((Ltorse-distMilieuBassin)/distMilieuBassin)*yBassin
                    if (coeffDirTronc(xBassin,yBassin,xMilieuCercle,yMilieuCercle)>0):
                        xDroite=xMilieuCercle
                        yDroite=yMilieuCercle
                    else:
                        xGauche=xMilieuCercle
                        yGauche=yMilieuCercle
            else:
                for j in range (100):
                    xMilieu=(xDroite+xGauche)/2
                    yMilieu=(yDroite+yGauche)/2
                    distMilieuBassin=distance(xMilieu,yMilieu,xBassin,yBassin)
                    xMilieuCercle=(Ltorse/distMilieuBassin)*xMilieu-((Ltorse-distMilieuBassin)/distMilieuBassin)*xBassin
                    yMilieuCercle=(Ltorse/distMilieuBassin)*yMilieu-((Ltorse-distMilieuBassin)/distMilieuBassin)*yBassin
                    if (coeffDirTronc(xBassin,yBassin,xMilieuCercle,yMilieuCercle)>0):
                        xGauche=xMilieuCercle
                        yGauche=yMilieuCercle
                    else:
                        xDroite=xMilieuCercle
                        yDroite=yMilieuCercle
            xTete=xGauche
            yTete=yGauche
            
            
            
    def difficutéTorse(xTete,yTete,xBassin,yBassin):
        return ((np.pi)/2-np.arctan(np.abs(float(coeffDirTronc(xTete,yTete,xBassin,yBassin))))/(np.pi/2))
    
    def difficultéÉquilibrePieds(pairOuImpair,e1,e2):
        if (pairOuImpair==0):
            return (e1+1)/2
        else:
            return (e1+e2)/2
    
    def difficultéÉquilibreMains(e3,e4): #lié au caractère bombé d'une prise
        return (e3+e4)/2
    
    LPhalanges=Lbras/9
    def difficultéTaillePrisesDeMain(t4,t3):
        if (t4>LPhalanges) :
            t4=LPhalanges
        if (t3>LPhalanges):
            t3=LPhalanges
        return (1-(t3+t4)/(2*LPhalanges))
        
    def coeffDir(P10,P20):
        return coeffDirTronc(P10[0],P10[1],P20[0],P20[1])
    
    def difficultéSouplesseJambes(PBassin,PPied1,PPied2):
        return (np.exp(np.arctan(np.abs(coeffDir(PBassin,PPied1)-coeffDir(PBassin,PPied2))))/np.exp(angleSouplesse))
    
    def difficultéMassique(p,t):
        imc=p/(t**2) 
        if (imc<=16.5):
            return 0.6
        elif (imc<=18.5):
            return 0.4
        elif (imc<=25):
            return 0.2
        elif (imc<=30):
            return 0.4
        elif (imc<=35):
            return 0.6
        elif (imc<=40):
            return 0.8
        else :
            return 1
            
    def difficultéAdhérence(pairOuImpair):
        return pairOuImpair
#    def coude(PMain, PTete,PAutreMain):
#        PMilieu=P((PMain[0]+PTete[0])/2,(PMain[1]+PTete[1])/2)
#        if (PMain[0]==PTete[0]):
#            if(PAutreMain[0]>=PTete[0]):
#                PCoude=P(PTete[0]-((Lbras/2)**2-distPoint(PMilieu,PMain)**2)**0.5,(PTete[1]+PMain[1])/2)
#                return PCoude
#            else:
#                PCoude=P(PTete[0]+((Lbras/2)**2-distPoint(PMilieu,PMain)**2)**0.5,(PTete[1]+PMain[1])/2)
#                return PCoude
#        else:
#            a=coeffDir(PMain,PTete)
#            if (PMain[0]>=PTete[0]):
#                if (PMain[1]>=PTete[1]):
#                    xCoude=PMilieu[0]+(abs((1/(1+1/a**2))*((Lbras)/2)**2-distPoint(PMilieu,PTete)**2))**0.5
#                    PCoude=P(xCoude,PMilieu[1]-(1/a)*(xCoude-PMilieu[0]))
#                    return PCoude
#                else:
#                    xCoude=PMilieu[0]-(abs((1/(1+1/a**2))*((Lbras)/2)**2-distPoint(PMilieu,PTete)**2))**0.5
#                    PCoude=P(xCoude,PMilieu[1]-(1/a)*(xCoude-PMilieu[0]))
#                    return PCoude 
#            else:
#                if (PMain[1]>=PTete[1]):
#                    xCoude=PMilieu[0]-(abs((1/(1+1/a**2))*((Lbras)/2)**2-distPoint(PMilieu,PTete)**2))**0.5
#                    PCoude=P(xCoude,PMilieu[1]-(1/a)*(xCoude-PMilieu[0]))
#                    return PCoude
#                else:
#                    xCoude=PMilieu[0]-(abs((1/(1+1/a**2))*((Lbras)/2)**2-distPoint(PMilieu,PTete)**2))**0.5
#                    PCoude=P(xCoude,PMilieu[1]-(1/a)*(xCoude-PMilieu[0]))
#                    return PCoude
    
#    x3Tete=(x3+xTete)/2
#    y3Tete=(y3+yTete)/2
#    if (y3>=PTete[1]):
#        if (x3<xTete):
#            coeffDirBras=(yTete-y3)/(xTete-x3)
#            distanceInterIntersections=2*((Lbras/2)**2-(distance(x3,y3,x3Tete,y3Tete))**2)**0.5
#            xIntersectionBasseCoude=x3Tete-(((distanceInterIntersections/2)**2)/((-1/coeffDirBras)**2+1))**0.5
#            yIntersectionBasseCoude=y3Tete-(-1/coeffDirBras)*(-xIntersectionBasseCoude+x3Tete)
#            #xIntersectionHaute=(((distanceInterIntersections/2)**2)/((1/coeffDirPied)**2+1))**0.5+x12
#            #yIntersectionHaute=(-1/coeffDirPied)*(xIntersectionHaute-x12)+y12
#        elif (x3>xTete):
#            coeffDirBras=(yTete-y3)/(xTete-x3)
#            distanceInterIntersections=2*((Lbras/2)**2-distance(x3,y3,x3Tete,y3Tete)**2)**0.5
#            xIntersectionBasseCoude=x3Tete+(((distanceInterIntersections/2)**2)/((-1/coeffDirBras)**2+1))**0.5
#            yIntersectionBasseCoude=y3Tete+(-1/coeffDirBras)*(xIntersectionBasseCoude-x3Tete)
#            #xIntersectionHaute=x3Tete-(((distanceInterIntersections/2)**2)/((1/coeffDirBras)**2+1))**0.5
#            #yIntersectionHaute=(-1/coeffDirPied)*(xIntersectionHaute-x12)+y12
#        else:
#            xIntersectionBasseCoude=PTete[0]
#            #xIntersectionHaute=x1
#            yIntersectionBasseCoude=maximum(y3,yTete)-Lbras/2
#            #yIntersectionHaute=minimum(y1,y2)+Ljambe
#    else:
#        if (x3>xTete):
#            coeffDirBras=(yTete-y3)/(xTete-x3)
#            distanceInterIntersections=2*((Lbras/2)**2-(distance(x3,y3,x3Tete,y3Tete))**2)**0.5
#            xIntersectionBasseCoude=x3Tete-(((distanceInterIntersections/2)**2)/((-1/coeffDirBras)**2+1))**0.5
#            yIntersectionBasseCoude=y3Tete-(-1/coeffDirBras)*(-xIntersectionBasseCoude+x3Tete)
#            #xIntersectionHaute=(((distanceInterIntersections/2)**2)/((1/coeffDirPied)**2+1))**0.5+x12
#            #yIntersectionHaute=(-1/coeffDirPied)*(xIntersectionHaute-x12)+y12
#        elif (x3<xTete):
#            coeffDirBras=(yTete-y3)/(xTete-x3)
#            distanceInterIntersections=2*((Lbras/2)**2-distance(x3,y3,x3Tete,y3Tete)**2)**0.5
#            xIntersectionBasseCoude=x3Tete+(((distanceInterIntersections/2)**2)/((-1/coeffDirBras)**2+1))**0.5
#            yIntersectionBasseCoude=y3Tete+(-1/coeffDirBras)*(xIntersectionBasseCoude-x3Tete)
#            #xIntersectionHaute=x3Tete-(((distanceInterIntersections/2)**2)/((1/coeffDirBras)**2+1))**0.5
#            #yIntersectionHaute=(-1/coeffDirPied)*(xIntersectionHaute-x12)+y12
#        else:
#            xIntersectionBasseCoude=x3
#            #xIntersectionHaute=x1
#            yIntersectionBasseCoude=maximum(y3,yTete)-Lbras/2
#            #yIntersectionHaute=minimum(y1,y2)+Ljambe
    
                
#    p3=P(x3,y3)
#    p4=P(x4,y4)
#    PTete=P(xTete,yTete)           
#    PCoude3=coude(p3,PTete,p4)
#    PCoude4=coude(p4,PTete,p3)
#    
#    def genou(PPied, PBassin,PAutrePied):
#        PMilieu=P((PPied[0]+PBassin[0])/2,(PPied[1]+PBassin[1])/2)
#        if (PPied[0]==PBassin[0]):
#            if(PAutrePied[0]>=PBassin[0]):
#                PGenou=P(PBassin[0]-((Ljambe/2)**2-distPoint(PMilieu,PPied)**2)**0.5,(PBassin[1]+PPied[1])/2)
#                return PGenou
#            else:
#                PGenou=P(PBassin[0]+(np.abs((Ljambe/2)**2-distPoint(PMilieu,PPied)**2))**0.5,(PBassin[1]+PPied[1])/2)
#                return PGenou
#        else:
#            a=coeffDir(PPied,PBassin)
#            if (a>=0):
#                xGenou=PMilieu[0]-(abs((1/(1+1/a**2))*((Ljambe)/2)**2-distPoint(PMilieu,PBassin)**2))**0.5
#                PGenou=P(xGenou,PMilieu[1]+(1/a)*(xGenou-PMilieu[0]))
#                return PGenou
#            else:
#                xGenou=PMilieu[0]+(abs((1/(1+1/a**2))*((Ljambe)/2)**2-distPoint(PMilieu,PBassin)**2))**0.5
#                PGenou=P(xGenou,PMilieu[1]+(1/a)*(xGenou-PMilieu[0]))
#                return PGenou
#                
#    PGenou1=genou(p1,PBassin,p2)
#    PGenou2=genou(p2,PBassin,p1)
    
    difficultéTotale=(1/7)*(difficultéAdhérence(pairOuImpair)+difficultéMassique(poids,taille)+difficutéTorse(xTete,yTete,xBassin,yBassin)+difficultéÉquilibrePieds(pairOuImpair,e1,e2)+difficultéÉquilibreMains(e3,e4)+difficultéTaillePrisesDeMain(t4,t3)+difficultéSouplesseJambes(PBassin,p1,p2))
    print ("difficultéTotale=",difficultéTotale) 
    
    print("Lbras2=",distance(x4,y4,xTete,yTete)) 
    print("Lbras1=",distance(x3,y3,xTete,yTete))
    print("LJambe1=",distance(x1,y1,xBassin,yBassin))
    print("LJambe2=",distance(x2,y2,xBassin,yBassin))
    print("LtorseFinal=",distance(xTete,yTete,xBassin,yBassin))
    print("nbPrises=",len(xPrises)," dernierePrise =",dernierePrise)
    #print(distPoint(PCoude3,PTete)+distPoint(PCoude3,p3))
    
    
    xTronc=[xBassin,xTete]
    yTronc=[yBassin,yTete]
    xJambe1=[xBassin,x1]
    yJambe1=[yBassin,y1]
    xJambe2=[xBassin,x2]
    yJambe2=[yBassin,y2]
    xBras1=[xTete,x3]
    #xBras3=[xTete,PCoude3[0],x3]
    yBras1=[yTete,y3]
    #yBras3=[yTete,PCoude3[1],y3]
    xBras2=[xTete,x4]
    #xBras4=[xTete,PCoude4[0],x4]
    yBras2=[yTete,y4]
    #yBras4=[yTete,PCoude4[1],y4]
    plt.axis([-2, 8, -1, 9])
    
    plt.plot(xPrises, yPrises, color='k', linestyle='',
    marker='o')
    plt.plot(xJambe1, yJambe1, color='#00008B', linestyle='-',
    marker='.',linewidth=2)
    
    plt.plot(xJambe2, yJambe2, color='#00008B', linestyle='-',
    marker='.',linewidth=2)
    plt.plot(xTronc, yTronc, color='#E0115F', linestyle='-', 
    marker='.',linewidth=2)
    plt.plot(xBras1, yBras1, color='#E0115F', linestyle='-',
    marker='.',linewidth=2)
#    plt.plot(xBras3, yBras3, color='#E0115F', linestyle='-',
#    marker='.',linewidth=2)
    plt.plot(xBras2, yBras2, color='#E0115F', linestyle='-',
    marker='.',linewidth=2)
#    plt.plot(xBras4, yBras4, color='#E0115F', linestyle='-',
#    marker='.',linewidth=2)
    plt.scatter(xTete, yTete+Ltorse/6,s=70, c='#D2691E')
    plt.show()
#    x1=x2
#    y1=y2
#    x2=x3
#    y2=y3
#    x3=x4
#    y3=y4
#    if (dernierePrise<len(xPrises)):
#        x4=xPrises[dernierePrise]
#        y4=yPrises[dernierePrise]
#    dernierePrise+=1
    if (pairOuImpair==0):
        x1=x2
        y1=y2
        e1=e2
        t1=t2
        x20=x3
        y20=y3
        t20=t3
        e20=e3
        x3=x4
        y3=y4
        e3=e4
        t3=t4
        if (dernierePrise<len(xPrises)):
            x4=xPrises[dernierePrise]
            y4=yPrises[dernierePrise]
            e4=instabilitésPrises[dernierePrise]
            t4=taillePrises[dernierePrise]
        if (coeffDir(PTete,P(x4,y4))>=0):
            x2=x1-Ljambe/2
            y2=y1+Ljambe*(1-np.sqrt(3)/2)
        if (coeffDir(PTete,P(x4,y4))<0):
            x2=x1+Ljambe/2
            y2=y1+Ljambe*(1-np.sqrt(3)/2)
#        if (x4>x3):
#            x2=x1-Ljambe/(np.sqrt(2))
#            y2=y1+Ljambe*(1-1/np.sqrt(2))
#        if (x4<=x3):
#            x2=x1+Ljambe/(np.sqrt(2))
#            y2=y1+Ljambe*(1-1/np.sqrt(2))
        dernierePrise+=1
        pairOuImpair+=1
    else:
        x2=x20
        y2=y20
        pairOuImpair+=-1
        e2=e20
        t2=t20
    