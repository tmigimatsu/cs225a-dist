/**
* \copyright
* TaCo - Task-Space Control Library.<br>
* Copyright (c) 2015-2016
*<br>
* TaCo is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*<br>
* TaCo is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*<br>
* You should have received a copy of the Lesser GNU Lesser General Public License
* along with TaCo.  If not, see <http://www.gnu.org/licenses/>.
<br>
Author: Brian Soe <bsoe@stanford.edu>
*/


#include "LBRRedisClient.h"
#include <friUdpConnection.h>
#include <friClientApplication.h>
#include <string.h>
#include <signal.h>

const int default_port = 30200;

int main (int argc, char** argv)
{
    // parse command line arguments
    if (argc > 1)
    {
        if ( strstr (argv[1],"help") != NULL)
        {
           printf(
                 "\nTest hold position code\n\n"
                 "\tCommand line arguments:\n"
                 "\t1) remote hostname (optional)\n"
                 "\t2) port ID (optional)\n"
           );
           return 1;
        }
    }
    char* hostname = (argc >= 2) ? argv[1] : NULL;
    int port = (argc >= 3) ? atoi(argv[2]) : default_port;

   // create new client
   KUKA::FRI::LBRRedisClient client;

   /***************************************************************************/
   /*                                                                         */
   /*   Standard application structure                                        */
   /*   Configuration                                                         */
   /*                                                                         */
   /***************************************************************************/

   // create new udp connection
   KUKA::FRI::UdpConnection connection;

   // pass connection and client to a new FRI client application
   KUKA::FRI::ClientApplication app(connection, client);
   
   // Connect client application to KUKA Sunrise controller.
   // Parameter NULL means: repeat to the address, which sends the data
   app.connect(port, hostname);

   /***************************************************************************/
   /*                                                                         */
   /*   Standard application structure                                        */
   /*   Execution mainloop                                                    */
   /*                                                                         */
   /***************************************************************************/

   // repeatedly call the step routine to receive and process FRI packets
   bool success = true;
   while (success)
   {
      success = app.step();
   }

   /***************************************************************************/
   /*                                                                         */
   /*   Standard application structure                                        */
   /*   Dispose                                                               */
   /*                                                                         */
   /***************************************************************************/

   // disconnect from controller
   app.disconnect();
   
   return 1;
}
