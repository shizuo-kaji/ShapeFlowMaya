# -*- coding: utf-8 -*-
# GUI for shapeFlow and ShapeMatching Maya plugins
# How To Use:
# 1. in script editor
# import plugin_deformer.ui_shapeFlow as ui
# reload(ui)
# ui.UI_Gradient()
#
# 2. Select target mesh and then shift+click to select the end mesh.
# 3. Create deformer from menu
# 4. Play animetion
# (Time slider is connected to the deformer to activate "commpute" method)

#  @author      Shizuo KAJI
#  @date        2013/May/13

#import debugmaya
#debugmaya.startDebug()

# Maya modules
import maya.cmds as cmds
import pymel.core as pm

# load plugin
deformerTypes = ["ShapeFlow","ShapeMatching"]

for type in deformerTypes:
    try:
        cmds.loadPlugin(type)
    except:
        print("Plugin %s already loaded" %(type))

# GUI
class UI_ShapeFlow:
    uiID = "ShapeFlow"
    title = "ShapeFlowPlugin"

    deformers = []
    
    ## Constructor
    def __init__(self):
        if pm.window(self.uiID, exists=True):
            pm.deleteUI(self.uiID)
        win = pm.window(self.uiID, title=self.title, menuBar=True)
        with win:
            pm.menu( label='Plugin', tearOff=True )
            for type in deformerTypes:
                pm.menuItem( label=type, c=pm.Callback( self.initPlugin, type) )
            self._parentLayout = pm.columnLayout( adj=True )
            with self._parentLayout:
                self.createUISet()

    def createUISet(self):
        self._childLayout = pm.columnLayout( adj=True )
        with self._childLayout:
            pm.text(l="Click target mesh, then shift+click end mesh")
            self.deformers = pm.ls(type=deformerTypes[0])
            for node in self.deformers:
                frameLayout = pm.frameLayout( label=node.name(), collapsable = True)
                with frameLayout:
                    pm.button( l="Del", c=pm.Callback( self.deleteNode, node))
                    pm.attrControlGrp( label="active", attribute= node.active)
                    pm.attrFieldSliderGrp(label="delta", min=0.001, max=5.0, attribute=node.delta)
                    pm.attrFieldSliderGrp(label="shapeMatching", min=0.1, max=10.0, attribute=node.smw)
        #
            self.deformers = pm.ls(type=deformerTypes[1])
            for node in self.deformers:
                frameLayout = pm.frameLayout( label=node.name(), collapsable = True)
                with frameLayout:
                    pm.button( l="Del", c=pm.Callback( self.deleteNode, node))
                    pm.attrControlGrp( label="active", attribute= node.active)
                    pm.attrFieldSliderGrp(label="delta", min=0.001, max=5.0, attribute=node.delta)
                    pm.attrFieldSliderGrp(label="stiffness", min=0.001, max=10.0, attribute=node.stf)
                    pm.attrFieldSliderGrp(label="attenuation", min=0.001, max=1.0, attribute=node.att)


    # delete deformer node
    def deleteNode(self,node):
        cmds.delete(node.name())
        self.updateUI()

    def initPlugin(self,type):
        meshes = pm.selected( type="transform" )
        if len(meshes)<2:
            return
        pm.select( meshes[-1])
        deformer = cmds.deformer(type=type)[0]
        shape=meshes[-2].getShapes()[0]
        cmds.connectAttr(shape+".outMesh", deformer+".startShape")
        cmds.connectAttr("time1.outTime", deformer+".slider")
        self.updateUI()

    def updateUI(self):
        pm.deleteUI( self._childLayout )
        pm.setParent(self._parentLayout)
        self.createUISet()
                             