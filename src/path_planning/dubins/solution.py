#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {Sebastian Barbas Laina}
# {student id}
# {ssbl@kth.se}

from dubins import *
from math import *

llamada=0
imprime=0



def mas_mejor_nodo(open):
    global nodo_mejor
    nodo_mejor=node(0,0,0.0, 2.0, Car(),0, 0)
    for i in range(0,len(open)):
        if open[i].h<nodo_mejor.h:
            nodo_mejor=open[i]
    

def sorting(vector, noded):
    global llamada, imprime
    prueba=0
    flag=0
    if not vector:
        vector.append(noded)
    else:
        for i in range(0,len(vector)):
            if vector[i].f>noded.f:
                vector.insert(i,noded)
                flag=1
                break
        if flag==0:
            vector.append(noded)
    if llamada>imprime:
        imprime=imprime+1000
        mas_mejor_nodo(vector)
        print('Longitud: ', len(vector))
        print(llamada)
        print(nodo_mejor.xp)
        print(nodo_mejor.yp)
        print(nodo_mejor.h)
    return vector



def reiteracion(x,y,thetha,car,phi):
    for i in range(0,20):
        xn,yn,thethan=step(car,x,y,thetha,phi)
        x=xn
        y=yn
        thetha=thethan
    return x,y,thetha



class node:

    def __init__(self,parnt, cost_arrive, xp, yp, car,angle, control):

        self.parent=parnt
        self.xf=car.xt
        self.yf=car.yt
        self.xp=xp
        self.yp=yp
        self.xdisc=round(xp,1)
        self.ydisc=round(yp,1)
        self.cost=cost_arrive+sqrt((car.xt-xp)**2+(car.yt-yp)**2)
        self.h=sqrt((car.xt-xp)**2+(car.yt-yp)**2)
        if self.parent!=0:
            self.g=cost_arrive+self.parent.g
        else:
            self.g=cost_arrive
        self.f=self.g+self.h
        self.thetha=angle
        self.control=control
       

    def iguales2(self,nodo_f):
        if self==nodo_f:
            return 1
        return 0


    def iguales(self,nodo_f):
        if self.xdisc==nodo_f.xdisc and self.ydisc==nodo_f.ydisc and nodo_f.thetha==self.thetha:
            return True
        return False

    def isfpos(self,xt, yt):
        if sqrt((self.xp-xt)**2+(self.yp-yt)**2)<=1.5:
            return True
        else:
            return False


def update_neighbours(car,open_close, close, node_parent):
    global llamada
    llamada=llamada+1
    control_trial=-pi/4
    while control_trial<=pi/4:
        flag=0
        #xn,yn,thetan=step(car, node_parent.xp, node_parent.yp, node_parent.thetha,control_trial)
        xn,yn,thetan=reiteracion(node_parent.xp,node_parent.yp,node_parent.thetha,car,control_trial)
        new_node=node(node_parent,sqrt((node_parent.xp-xn)**2+(node_parent.yp-yn)**2),xn,yn,car,thetan,control_trial)
        for i in range(0, len(close)):
            if close[i].iguales2(new_node):
                break
            if i==(len(close)-1):
                if car._environment.safe(xn, yn):
                    for j in range(0, len(open_close)):
                        if open_close[j].iguales2(new_node):
                            if open_close[j].g>new_node.g:
                                flag=1
                                break
                    if flag==0:
                        open_close.append(new_node)

                                
        else:
            close.append(new_node)
        control_trial=control_trial+pi/4
    return





        


def update_control_times(controls,times,noded):
    time=0
    camino=[]
    time_aid=[0]
    while noded.parent!=0:
        time=time+0.2
        camino.append(noded.control)
        noded=noded.parent
        time_aid.append(time)
    return camino[::-1], time_aid

def poping(open,noded):
    integ=0
    for i,nod in enumerate(open):
        if sqrt((nod.xp-nod.xf)**2+(nod.yp-nod.yf)**2)<sqrt((noded.xp-noded.xf)**2+(noded.yp-noded.yf)**2):
            noded=nod
            integ=i
    return integ, nod




def solution(car):
    ''' <<< write your code below >>> '''
    variable_break=0
    controls=[0]
    times=[0,1]
    ns=node(0,0,car.x0, car.y0, car,0, 0)
 
    open=[ns]
    close=[]
    while len(open)>0:
        variable_break=variable_break+1
        noded=open[0]
        integ,noded=poping(open,noded)
        noded=open.pop(integ)
        close.append(noded)
        
        if noded.isfpos(car.xt, car.yt):
            print('LLegaaaaaaaaaa')
            controls, times=update_control_times(controls, times, noded)
            return controls, times
        else:
            update_neighbours(car,open,close,noded) 
        if variable_break>500:
            break




    ''' <<< write your code below >>> '''

    return controls, times
