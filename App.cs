using Bridge;
using Newtonsoft.Json;
using Bridge.Html5;
using BridgeGeneralRoboticsToolbox;
using GeneralRoboticsToolboxTests;
using System;


namespace BridgeGeneralRoboticsToolbox
{
    public class App
    {
        public static void Main()
        {
            UnitTest1 tests = new UnitTest1();
            
            Console.WriteLine("Welcome to Bridge.NET");

            // After building (Ctrl + Shift + B) this project, 
            // browse to the /bin/Debug or /bin/Release folder.

            // A new bridge/ folder has been created and
            // contains your projects JavaScript files. 

            // Open the bridge/index.html file in a browser by
            // Right-Click > Open With..., then choose a
            // web browser from the list

            // This application will then run in the browser.
        }
    }
}