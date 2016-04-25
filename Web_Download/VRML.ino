void vrmlHeader()
{
  if (myFile)
  {
    myFile.println("#VRML V2.0 utf8");
    myFile.println("#Projet LIDAR : Fabien SANTOS-CESSAC Loïc OLÇOMENDY\n");
    myFile.println("NavigationInfo {type \"EXAMINE\"}");
    myFile.println("Viewpoint {");
    myFile.println("position    0 0 0 ");
    myFile.println("description \"Position 1\" ");
    myFile.println("fieldOfView 0.55");
    myFile.println("orientation 0 0 1 0");
    myFile.println("}");
    myFile.println("Shape {");
    myFile.println("geometry IndexedFaceSet {");
    myFile.println("solid FALSE");
    myFile.println("coord Coordinate {");
    myFile.print("point [");
  }
}
