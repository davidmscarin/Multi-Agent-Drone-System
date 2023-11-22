import wireframe
import pygame
import os
import numpy as np

class EnvironmentViewer:
    #displays 3D objects on a 2D Pygame screen

    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption('Environment Display')
        self.background = (100,100,100)

        self.wireframes = {}
        self.displayNodes = True
        self.hub_im_size = (75,75)
        self.drone_im_size = (70,70)
        self.drone_size_list = [0.5,0.55,0.6,0.65,0.7,0.75,0.8,0.85,0.9,0.95,1]
        self.package_im_size = (50,50)
        self.package_size_list = [0.5,0.55,0.6,0.65,0.7,0.75,0.8,0.85,0.9,0.95,1]
        self.flag_im_size = (35,50)
        self.flag_size_list = [0.5,0.55,0.6,0.65,0.7,0.75,0.8,0.85,0.9,0.95,1]

    def addWireframe(self, name, wireframe):
        #add a named wireframe object

        self.wireframes[name] = wireframe

    def run(self):
        #create a pygame screen until it is closed
        
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    
            self.display()  
        
    def display(self):
        #draw the wireframes on the screen

        self.screen.fill(self.background)
        im_path = os.getcwd()

        for wireframe in self.wireframes.values():
            temp_path = os.getcwd()
            for node in wireframe.nodes:
                if node.type == 0:      # HUB
                    im_path = temp_path+'\\images\\hub.png'
                    Image = pygame.image.load(im_path)
                    Image = pygame.transform.scale(Image, self.hub_im_size)
                elif node.type == 3:
                    im_path = temp_path+'\\images\\flag.png'
                    mult = self.flag_size_list[node.real_z]
                    im_size = tuple(mult*np.array(self.flag_im_size))
                    Image = pygame.image.load(im_path)
                    Image = pygame.transform.scale(Image, im_size)
                elif node.type == 2:    # PACKAGE
                    im_path = temp_path+'\\images\\package.png'
                    mult = self.package_size_list[node.real_z]
                    im_size = tuple(mult*np.array(self.package_im_size))
                    Image = pygame.image.load(im_path)
                    Image = pygame.transform.scale(Image, im_size)
                elif node.type == 1:    # DRONE
                    im_path = temp_path+'\\images\\drone.png'
                    mult = self.drone_size_list[node.real_z]
                    im_size = tuple(mult*np.array(self.drone_im_size))
                    Image = pygame.image.load(im_path)
                    Image = pygame.transform.scale(Image, im_size)
                self.screen.blit(Image, (node.x,node.y))
        # update the display with the new coordinates
        pygame.display.flip()