


int clientRequest(EthernetClient client)
{
  if (client)
    if (client.connected())
      if (client.available())
      {
        while (client.available())
        {
          client.read();//flush
        }
        return 1;
      }
  client.stop();
  return 0;
}

void webHomePage(EthernetClient client)
{
  client.println("HTTP/1.1 200 OK"); //send new page
  client.println("Content-Type: text/html");
  client.println();     
  client.println("<HTML>");
  client.println("<HEAD>");
  client.println("<meta name='apple-mobile-web-app-capable' content='yes' />");
  client.println("<meta name='apple-mobile-web-app-status-bar-style' content='black-translucent' />");
  client.println("<link rel='stylesheet' type='text/css' href='http://randomnerdtutorials.com/ethernetcss.css' />");
  client.println("<TITLE>3D LIDAR Scanner</TITLE>");
  client.println("</HEAD>");
  client.println("<BODY>");
  client.println("<H1>3D LIDAR Scanner</H1>");
  client.println("<hr />");
  client.println("<br />");  
  client.println("<H2>Controls:</H2>");
  client.println("<br />");  
  client.println("<a href=\"/?3Dscan\"\">Scan</a>");
  client.println("<a href=\"/?vrmlFile\"\">Download</a><br />");   
  client.println("<br />"); 
  client.println("</BODY>");
  client.println("</HTML>");
  
  delay(1);
  client.stop();
}

void webScanPage(EthernetClient client,int percentage)//displays the scanning progress
{
  client.println("HTTP/1.1 200 OK"); //send new page
  client.println("Content-Type: text/html");
  client.println("Connection: close");  // the connection will be closed after completion of the response
  client.println();
  client.println("<HTML>");
  client.println("<HEAD>");
  client.println("<meta http-equiv=\"refresh\" content=\"1; url=http://169.254.28.77/\" />");
  client.println("<meta name='apple-mobile-web-app-capable' content='yes' />");
  client.println("<meta name='apple-mobile-web-app-status-bar-style' content='black-translucent' />");
  client.println("<link rel='stylesheet' type='text/css' href='http://randomnerdtutorials.com/ethernetcss.css' />");
  client.println("<TITLE>3D LIDAR Scanner</TITLE>");
  client.println("</HEAD>");
  client.println("<BODY>");
  client.println("<H1>3D LIDAR Scanner</H1>");
  client.println("<hr />");
  client.println("<br />");
  client.print("<H2>Scanning: ");client.print(percentage);client.println("%</H2>");
  
  client.println("</BODY>");
  client.println("</HTML>");
  
  delay(1);
  client.stop();
}

void webScanOnly(EthernetClient client)
{
  client.println("HTTP/1.1 200 OK"); //send new page
  client.println("Content-Type: text/html");
  client.println();     
  client.println("<HTML>");
  client.println("<HEAD>");
  client.println("<meta name='apple-mobile-web-app-capable' content='yes' />");
  client.println("<meta name='apple-mobile-web-app-status-bar-style' content='black-translucent' />");
  client.println("<link rel='stylesheet' type='text/css' href='http://randomnerdtutorials.com/ethernetcss.css' />");
  client.println("<TITLE>3D LIDAR Scanner</TITLE>");
  client.println("</HEAD>");
  client.println("<BODY>");
  client.println("<H1>3D LIDAR Scanner</H1>");
  client.println("<hr />");
  client.println("<br />");  
  client.println("<H2>File not found, please scan to create a new one:</H2>");
  client.println("<br />");  
  client.println("<a href=\"/?3Dscan\"\">Scan</a>"); 
  client.println("<br />"); 
  client.println("</BODY>");
  client.println("</HTML>");
            
  delay(1);
  client.stop();
}

void WebScanComplete(EthernetClient client)
{
  client.println("HTTP/1.1 200 OK"); //send new page
  client.println("Content-Type: text/html");
  client.println();     
  client.println("<HTML>");
  client.println("<HEAD>");
  client.println("<meta http-equiv=\"refresh\" content=\"0; url=http://http://169.254.28.77/\" />");
  client.println("<meta name='apple-mobile-web-app-capable' content='yes' />");
  client.println("<meta name='apple-mobile-web-app-status-bar-style' content='black-translucent' />");
  client.println("<link rel='stylesheet' type='text/css' href='http://randomnerdtutorials.com/ethernetcss.css' />");
  client.println("<TITLE>3D LIDAR Scanner</TITLE>");
  client.println("</HEAD>");
  client.println("<BODY>");
  client.println("<H1>Scan done</H1>");
  client.println("<hr />");
  client.println("<br />");  
  client.println("<H2>Controls:</H2>");
  client.println("<br />");  
  client.println("<a href=\"/?3Dscan\"\">Scan</a>");
  client.println("<a href=\"/?vrmlFile\"\">Download</a><br />");   
  client.println("<br />"); 
  client.println("<br />"); 
  client.println("<tt>Scan complete!</tt>"); 
  client.println("</BODY>");
  client.println("</HTML>");
          
  delay(1);
  client.stop();
}

void webFileDownload(EthernetClient client)
{
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: model/vrml");//Allows the browser to view directly?
  client.println("Content-Disposition: attachment; filename=\"LIDAR.wrl\"");
  client.println();
  
  delay(1);
  client.stop();
}

void web404(EthernetClient client)
{
  client.println("HTTP/1.1 404 Not Found");
  client.println("Content-Type: text/html");
  client.println();
  client.println("<h2>Invalid command</h2>");
  
  delay(1);
  client.stop();
}
