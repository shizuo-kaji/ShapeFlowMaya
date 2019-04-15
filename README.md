Shape Matching plugins for Maya
/**
 * @brief Shape Matching plugins for Maya
 * @section LICENSE The MIT License
 * @section requirements:  Eigen 3:  http://eigen.tuxfamily.org/
 * @section Autodesk Maya: http://www.autodesk.com/products/autodesk-maya/overview
 * @section AffineLib: https://github.com/shizuo-kaji/AffineLib
 * @version 0.10
 * @date  3/Nov/2013
 * @author Shizuo KAJI
 */

How to compile:
Get devkit at https://www.autodesk.com/developmaya
Look at the included Xcode project file.
For Windows users, please refer to Autodesk's web page.

How to use:
put the plugins in "MAYA_PLUG_IN_PATH"
put the UI python script in "MAYA_SCRIPT_PATH"
open script editor in Maya and type in the following Python command:
#
import ui_shapeFlow as ui
ui.UI_ShapeFlow()
#