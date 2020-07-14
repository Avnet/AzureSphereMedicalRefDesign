/* Copyright (c) Microsoft Corporation. All rights reserved.
   Licensed under the MIT License. */

// This sample C application for Azure Sphere demonstrates Azure IoT SDK C APIs
// The application uses the Azure IoT SDK C APIs to
// 1. Use the buttons to trigger sending telemetry to Azure IoT Hub/Central.
// 2. Use IoT Hub/Device Twin to control an LED.

// You will need to provide four pieces of information to use this application, all of which are set
// in the app_manifest.json.
// 1. The Scope Id for your IoT Central application (set in 'CmdArgs')
// 2. The Tenant Id obtained from 'azsphere tenant show-selected' (set in 'DeviceAuthentication')
// 3. The Azure DPS Global endpoint address 'global.azure-devices-provisioning.net'
//    (set in 'AllowedConnections')
// 4. The IoT Hub Endpoint address for your IoT Central application (set in 'AllowedConnections')

#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>
#include <stdarg.h>
#include <errno.h>
#include <ctype.h>
#include <pthread.h> 
#include <sys/time.h>

// applibs_versions.h defines the API struct versions to use for applibs APIs.
#include "applibs_versions.h"
#include <applibs/log.h>
#include <applibs/networking.h>
#include <applibs/gpio.h>
#include <applibs/i2c.h>
#include <applibs/storage.h>
#include <applibs/eventloop.h>
#include <applibs/application.h>
#include <applibs/wificonfig.h>
#include <applibs/uart.h>

// By default, this example targets hardware that follows the Avnet MT3620
// Starter Kit.
#include <hw/avnet_mt3620_sk.h>
#include "build_options.h"

// Azure IoT SDK
#include <iothub_client_core_common.h>
#include <iothub_device_client_ll.h>
#include <iothub_client_options.h>
#include <iothubtransportmqtt.h>
#include <iothub.h>
#include <azure_sphere_provisioning.h>

#include "eventloop_timer_utilities.h"
#include "parson.h" // used to parse Device Twin messages.
#include "oled.h"
#include "hr4.h"

/// <summary>
/// Exit codes for this application. These are used for the
/// application exit code. They must all be between zero and 255,
/// where zero is reserved for successful termination.
/// </summary>
typedef enum {
    ExitCode_Success = 0,
    ExitCode_TermHandler_SigTerm,
    ExitCode_Main_EventLoopFail,
    ExitCode_ButtonTimer_Consume,
    ExitCode_AzureTimer_Consume,
    ExitCode_Init_EventLoop,
    ExitCode_Init_MessageButton,
    ExitCode_Init_OrientationButton,
    ExitCode_Init_TwinStatusLed,
    ExitCode_Init_ButtonPollTimer,
    ExitCode_Init_AzureTimer,
    ExitCode_IsButtonPressed_GetValue,
    ExitCode_Init_SendTimer,
    ExitCode_Init_Connection,
    ExitCode_Init_SetSockOpt,
    ExitCode_Init_RegisterIo,
    ExitCode_Init_UartOpen,
    ExitCode_TimerHandler_Consume,
    ExitCode_SendMsg_Send,
    ExitCode_SocketHandler_Recv,
    ExitCode_Init_NetworkPollTimer,
    ExitCode_Init_HR4TimerPollTimer,
    ExitCode_NetworkTimer_Consume,
    ExitCode_UartEvent_Read,
    ExitCode_SendMessage_Write,
} ExitCode;

// File Descriptors for the OLED
int i2cFd = -1;

// Intercore communications socket file descriptor
static int sockFd = -1;

// Uart file descriptor
static int uartFd = -1;

// Global exitCode variable
static volatile sig_atomic_t exitCode = ExitCode_Success;

// Component ID for the realtime application
static const char rtAppComponentId[] = "1ee4e525-4456-4a63-a2af-e2a2cb1518e9";

bool RTCore_status;

// Flag that controls how the application behaves after a successfull test
// if the flag is true the applicaiton goes into a new test pass after displaying
// the results for 30 seconds.  If false, then the user must restart the next test
// with the A button or by using the cloud control.
bool autoRestartHRTest = false;

static void TerminationHandler(int signalNumber);
#ifdef SEND_CANNED_MESSAGES_TO_RT_APP
static void SendTimerEventHandler(EventLoopTimer* timer);
static void SendMessageToRTApp(void);
#endif 
static void SocketEventHandler(EventLoop* el, int fd, EventLoop_IoEvents events, void* context);

// Azure IoT Hub/Central defines
#define SCOPEID_LENGTH 20
static char scopeId[SCOPEID_LENGTH]; // ScopeId for the Azure IoT Central application, set in
                                     // app_manifest.json, CmdArgs

// Azure IoT Hub/Central routines
static IOTHUB_DEVICE_CLIENT_LL_HANDLE iothubClientHandle = NULL;
static const int keepalivePeriodSeconds = 20;
static bool iothubAuthenticated = false;
static void SendMessageCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void *context);
static int DirectMethodCall(const char*, const char*, size_t, char**, size_t*);
static void TwinCallback(DEVICE_TWIN_UPDATE_STATE updateState, const unsigned char *payload,
                         size_t payloadSize, void *userContextCallback);
static void TwinReportBoolState(const char *propertyName, bool propertyValue);
static void TwinReportString(const unsigned char* jsonMessage);
static void ReportStatusCallback(int result, void *context);
static const char *GetReasonString(IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason);
static const char *getAzureSphereProvisioningResultString(
    AZURE_SPHERE_PROV_RETURN_VALUE provisioningResult);
static void SendTelemetry(const unsigned char *key, const unsigned char *value);
void SendJSONTelemetry(const unsigned char* jsonMessage);
static void SetupAzureClient(void);

// Utility routines
void startHR4();

// Application Initialization/Cleanup
static ExitCode InitPeripheralsAndHandlers(void);
static void CloseFdAndPrintError(int fd, const char *fdName);
static void ClosePeripheralsAndHandlers(void);

// UART routine
static void SendUartMessage(int uartFd, const char* dataToSend);

// Button file descriptors
static int buttonAGpioFd = -1;
static int buttonBGpioFd = -1;

// OLED data
network_var network_data;

// LED
static int deviceTwinStatusLedGpioFd = -1;
static bool statusLedOn = false;

// Timer / polling
static EventLoop* eventLoop = NULL;
static EventLoopTimer* buttonPollTimer = NULL;
static EventLoopTimer* azureTimer = NULL;
static EventLoopTimer* updateNetworkPollTimer = NULL;
static EventLoopTimer* sendTimer = NULL;
static EventRegistration* socketEventReg = NULL;
EventLoopTimer* hr4Timer = NULL;
EventRegistration* uartEventReg = NULL;

// Variable to manage dynamic poll period
static int azureIoTPollPeriodSeconds = -1;

// Azure IoT default poll periods
static const int AzureIoTDefaultPollPeriodSeconds = 5;
static const int AzureIoTMinReconnectPeriodSeconds = 60;
static const int AzureIoTMaxReconnectPeriodSeconds = 10 * 60;

// Button state variables
static GPIO_Value_Type buttonAState = GPIO_Value_High;
static GPIO_Value_Type buttonBState = GPIO_Value_High;

// Button specific routines
static void ButtonPollTimerEventHandler(EventLoopTimer *timer);
static bool IsButtonPressed(int fd, GPIO_Value_Type *oldState);
static void ButtonAHandler(void);
static void ButtonBHandler(void);

// Event handlers
static void AzureTimerEventHandler(EventLoopTimer *timer);
static void NetworkPollTimerEventHandler(EventLoopTimer* timer);
static void UartEventHandler(EventLoop* el, int fd, EventLoop_IoEvents events, void* context);

/// <summary>
///     Allocates and formats a string message on the heap.
/// </summary>
/// <param name="messageFormat">The format of the message</param>
/// <param name="maxLength">The maximum length of the formatted message string</param>
/// <returns>The pointer to the heap allocated memory.</returns>
static void* SetupHeapMessage(const char* messageFormat, size_t maxLength, ...)
{
    va_list args;
    va_start(args, maxLength);
    char* message =
        malloc(maxLength + 1); // Ensure there is space for the null terminator put by vsnprintf.
    if (message != NULL) {
        vsnprintf(message, maxLength, messageFormat, args);
    }
    va_end(args);
    return message;
}

/// <summary>
/// Initialize the i2c interface:  i2c is used for the OLED
/// </summary>
void init_i2c(void) {

    // Open the i2c interface
    i2cFd = I2CMaster_Open(AVNET_MT3620_SK_ISU2_I2C);
    if (i2cFd < 0) {
        Log_Debug("ERROR: I2CMaster_Open: errno=%d (%s)\n", errno, strerror(errno));
        return;
    }

    // Set the i2c bus speed
    int result = I2CMaster_SetBusSpeed(i2cFd, I2C_BUS_SPEED_STANDARD);
    if (result != 0) {
        Log_Debug("ERROR: I2CMaster_SetBusSpeed: errno=%d (%s)\n", errno, strerror(errno));
        return;
    }

    // Set the i2c timeout
    result = I2CMaster_SetTimeout(i2cFd, 100);
    if (result != 0) {
        Log_Debug("ERROR: I2CMaster_SetTimeout: errno=%d (%s)\n", errno, strerror(errno));
        return;
    }

    // Start OLED
    if (oled_init())
    {
        Log_Debug("OLED not found!\n");
    }
    else
    {
        Log_Debug("OLED found!\n");

        // Draw AVNET logo
        oled_draw_logo();
    }
}

/// <summary>
/// Network poll timer event:  Read the current network status
/// </summary>
/// This routine peroidically reads the wifi status for display on the OLED
static void NetworkPollTimerEventHandler(EventLoopTimer* timer)
{
    // Keep track of the first time into the routine
    static bool firstPass = true;
    
    char ssid[128];
    uint32_t frequency;
    char bssid[20];

    if (ConsumeEventLoopTimerEvent(timer) != 0) {
        exitCode = ExitCode_NetworkTimer_Consume;
        return;
    }

    // We want to update the data structures if we're currently
    // displaying the network status OLED screen, or if this is the first time into this routine
    if ((oled_state == NETWORK_STATUS) || (firstPass == true)) {

        firstPass = false;

        // Clear the ssid array
        memset(ssid, 0, 128);

        // Get the network status
        WifiConfig_ConnectedNetwork network;
        int result = WifiConfig_GetCurrentNetwork(&network);
        if (result != 0 && errno != ENODATA) {
            Log_Debug("\nERROR: WifiConfig_GetCurrentNetwork failed: %s (%d).\n", strerror(errno),
                errno);
        }

        if (result < 0)
        {
            // Log_Debug("INFO: Not currently connected to a WiFi network.\n");
            strncpy(network_data.SSID, "Not Connected", 20);
            network_data.frequency_MHz = 0;
            network_data.rssi = 0;
        }
        else
        {
            // Log_Debug("wifi connected!\n");
            frequency = network.frequencyMHz;
            snprintf(bssid, 18, "%02x:%02x:%02x:%02x:%02x:%02x",
                network.bssid[0], network.bssid[1], network.bssid[2],
                network.bssid[3], network.bssid[4], network.bssid[5]);

            if (strncmp(ssid, (char*)&network.ssid, network.ssidLength) != 0) /*|| !networkConfigSent)*/ {

                // Clear the ssid array
                memset(ssid, 0, 128);
                strncpy(ssid, network.ssid, network.ssidLength);
                Log_Debug("SSID: %s\n", ssid);
                Log_Debug("Frequency: %dMHz\n", frequency);
                Log_Debug("bssid: %s\n", bssid);
            }

            memset(network_data.SSID, 0, WIFICONFIG_SSID_MAX_LENGTH);
            if (network.ssidLength <= SSID_MAX_LEGTH)
            {
                strncpy(network_data.SSID, network.ssid, network.ssidLength);
            }
            else
            {
                strncpy(network_data.SSID, network.ssid, SSID_MAX_LEGTH);
            }

            network_data.frequency_MHz = network.frequencyMHz;

            network_data.rssi = network.signalRssi;
        }

        // If the Network display is the current display, then update the OLED
        if (oled_state == NETWORK_STATUS) {
         
            // Update the display
            update_network();
        }
    }
}

#ifdef SEND_CANNED_MESSAGES_TO_RT_APP
/// <summary>
///     Handle send timer event by writing data to the real-time capable application.
/// </summary>
static void SendTimerEventHandler(EventLoopTimer* timer)
{
    if (ConsumeEventLoopTimerEvent(timer) != 0) {
        exitCode = ExitCode_TimerHandler_Consume;
        return;
    }

    SendMessageToRTApp();
}
#endif 

#ifdef SEND_CANNED_MESSAGES_TO_RT_APP
/// <summary>
///     Helper function for TimerEventHandler sends message to real-time capable application.
/// </summary>
static void SendMessageToRTApp(void)
{
    // Declare an interator to keep track of how many times we enter the routine
    static int cnt = 0;

    // Declare a boolean to keep track of which command to send, start or stop
    static bool sendStartCmd = true;

    // Define both messages
    static char startMsg[] = "{\"RTAppCommand\":\"START\"}\0";
    static char stopMsg[] = "{\"RTAppCommand\":\"STOP\"}\0";
    char* txMessage;

    // Select the message to send based off the boolean
    txMessage = sendStartCmd ? startMsg : stopMsg;

    // Toggle the boolean so we send the other command next time
    sendStartCmd = !sendStartCmd;

    // Reset the count
    cnt = 0;

    Log_Debug("\n\nSending: %s\n\n", txMessage);

    int bytesSent = send(sockFd, txMessage, strlen(txMessage), 0);
    if (bytesSent == -1) {
        Log_Debug("ERROR: Unable to send message: %d (%s)\n", errno, strerror(errno));
        exitCode = ExitCode_SendMsg_Send;
        return;
    }
}

#endif 

/// <summary>
///     Handle socket event by reading incoming data from real-time capable application.
/// </summary>
static void SocketEventHandler(EventLoop* el, int fd, EventLoop_IoEvents events, void* context)
{
    // Read messages from real-time capable application.
    // If the RTApp has sent more than 64 bytes, then truncate.
    char rxBuf[64];
    int bytesReceived = recv(fd, rxBuf, sizeof(rxBuf), 0);

    if (bytesReceived == -1) {
        Log_Debug("ERROR: Unable to receive message: %d (%s)\n", errno, strerror(errno));
        exitCode = ExitCode_SocketHandler_Recv;
        return;
    }

    Log_Debug("Received %d bytes: ", bytesReceived);
    for (int i = 0; i < bytesReceived; ++i) {
        Log_Debug("%c", isprint(rxBuf[i]) ? rxBuf[i] : '.');
    }
    Log_Debug("\n");

    // Parse the JSON message
    JSON_Value* rootPtr = NULL;
    rootPtr = json_parse_string(rxBuf);
    if (rootPtr == NULL) {
        Log_Debug("WARNING: Cannot parse the string as JSON content.\n");

        // If this is not valid JSON then exit!
        return;
    }

    // Get a pointer to the root object
    JSON_Object* rootObject = json_value_get_object(rootPtr);
    if (rootObject == NULL) {
        Log_Debug("Error parsing JSON/n");
        return;
    }

    // Try to get a pointer to the "oled" object.  If it does not exist, then assume this
    // is a telemetry message and send the JSON string up to Azure IoT Hub as telemetry
    // Note that we've already validated that this is valid JSON
    JSON_Object* oledProperties = json_object_dotget_object(rootObject, "oled");
    if (oledProperties == NULL) {

        // Call the routine to send the JSON as telemetry
        SendJSONTelemetry(rxBuf);

    }
    // This is an OLED message, parse out the line number and data to display
    else
    {
        // Handle the oled Properties here.
        uint8_t oledLineNumber = atoi(json_object_get_string(oledProperties, "ln"));
        Log_Debug("OLED Line Number: %d\n", oledLineNumber);

        char* oledText = json_object_get_string(oledProperties, "str");
        Log_Debug("OLED text to disaply: %s\n", oledText);
        uint8_t oledTextLen = strlen(oledText);
        if (oledTextLen > OLED_LINE_LEN) {
            Log_Debug("Trunkating message: %d, %d\n", oledTextLen, OLED_LINE_LEN);
            oledTextLen = OLED_LINE_LEN;
        }

        // Copy the new text into oled message data structure
        strncpy(rtOledMsg[oledLineNumber], oledText, oledTextLen);
        
        // Null terminate the string in the array. The display code uses null to
        // identify the end of a string.  Without the null, we'll keep writting 
        // the entire buffer to the OLED.
        rtOledMsg[oledLineNumber][oledTextLen] = '\0';

        // If the current screen is the RTApplication screen, then call the routine
        // to refresh the display with the new data.
        if (oled_state == RTAPP_MESSAGE) {

            // Refresh the OLED display to show the new data
            update_oled();
        }
    }

    // Free any memory allocated by the parson library
    json_value_free(rootPtr);
}

/// <summary>
///     Signal handler for termination requests. This handler must be async-signal-safe.
/// </summary>
static void TerminationHandler(int signalNumber)
{
    // Don't use Log_Debug here, as it is not guaranteed to be async-signal-safe.
    exitCode = ExitCode_TermHandler_SigTerm;
}

/// <summary>
///     Main entry point for this sample.
/// </summary>
int main(int argc, char *argv[])
{
    Log_Debug("Health Care Application starting.\n");

    bool isNetworkingReady = false;
    if ((Networking_IsNetworkingReady(&isNetworkingReady) == -1) || !isNetworkingReady) {
        Log_Debug("WARNING: Network is not ready. Device cannot connect until network is ready.\n");
    }

    if (argc == 2) {
        Log_Debug("Setting Azure Scope ID %s\n", argv[1]);
        strncpy(scopeId, argv[1], SCOPEID_LENGTH);
    } else {
        Log_Debug("ScopeId needs to be set in the app_manifest CmdArgs\n");
        return -1;
    }

    exitCode = InitPeripheralsAndHandlers();

    // Main loop
    while (exitCode == ExitCode_Success) {
        EventLoop_Run_Result result = EventLoop_Run(eventLoop, -1, true);
        // Continue if interrupted by signal, e.g. due to breakpoint being set.
        if (result == EventLoop_Run_Failed && errno != EINTR) {
            exitCode = ExitCode_Main_EventLoopFail;
        }
    }

    ClosePeripheralsAndHandlers();
    Log_Debug("Application exiting.\n");
    return exitCode;
}

/// <summary>
/// Button timer event:  Check the status of the buttons
/// </summary>
static void ButtonPollTimerEventHandler(EventLoopTimer *timer)
{
    if (ConsumeEventLoopTimerEvent(timer) != 0) {
        exitCode = ExitCode_ButtonTimer_Consume;
        return;
    }
    ButtonAHandler();
    ButtonBHandler();
}

/// <summary>
/// Azure timer event:  Check connection status and send telemetry
/// </summary>
static void AzureTimerEventHandler(EventLoopTimer *timer)
{
    if (ConsumeEventLoopTimerEvent(timer) != 0) {
        exitCode = ExitCode_AzureTimer_Consume;
        return;
    }

#if (defined(IOT_CENTRAL_APPLICATION))

    bool isNetworkReady = false;
    if (Networking_IsNetworkingReady(&isNetworkReady) != -1) {
        if (isNetworkReady && !iothubAuthenticated) {
            SetupAzureClient();
        }
    } else {
        Log_Debug("Failed to get Network state\n");
    }

    if (iothubAuthenticated) {
        IoTHubDeviceClient_LL_DoWork(iothubClientHandle);
    }
#endif 
}

/// <summary>
///     Set up SIGTERM termination handler, initialize peripherals, and set up event handlers.
/// </summary>
/// <returns>
///     ExitCode_Success if all resources were allocated successfully; otherwise another
///     ExitCode value which indicates the specific failure.
/// </returns>
static ExitCode InitPeripheralsAndHandlers(void)
{
    struct sigaction action;
    memset(&action, 0, sizeof(struct sigaction));
    action.sa_handler = TerminationHandler;
    sigaction(SIGTERM, &action, NULL);

    eventLoop = EventLoop_Create();
    if (eventLoop == NULL) {
        Log_Debug("Could not create event loop.\n");
        return ExitCode_Init_EventLoop;
    }

#ifdef SEND_CANNED_MESSAGES_TO_RT_APP
    // Register a one second timer to send a message to the RTApp.
    static const struct timespec sendPeriod = { .tv_sec = CANNED_MESSAGES_PERIOD_SECONDS, .tv_nsec = 0 };
    sendTimer = CreateEventLoopPeriodicTimer(eventLoop, &SendTimerEventHandler, &sendPeriod);
    if (sendTimer == NULL) {
        return ExitCode_Init_SendTimer;
    }
#endif 

#ifdef M0_INTERCORE_COMMS
    // Open a connection to the RTApp.
    sockFd = Application_Connect(rtAppComponentId);
    if (sockFd == -1) {
        Log_Debug("ERROR: Unable to create socket: %d (%s)\n", errno, strerror(errno));
        return ExitCode_Init_Connection;
    }
    // We just opened a socket to the RTApplication, so it's there and responding.
    RTCore_status = true;

    // Set timeout, to handle case where real-time capable application does not respond.
    static const struct timespec recvTimeout = { .tv_sec = 5, .tv_nsec = 0 };
    int result = setsockopt(sockFd, SOL_SOCKET, SO_RCVTIMEO, &recvTimeout, sizeof(recvTimeout));
    if (result == -1) {
        Log_Debug("ERROR: Unable to set socket timeout: %d (%s)\n", errno, strerror(errno));
        return ExitCode_Init_SetSockOpt;
    }

    // Register handler for incoming messages from real-time capable application.
    socketEventReg = EventLoop_RegisterIo(eventLoop, sockFd, EventLoop_Input, SocketEventHandler,
        /* context */ NULL);
    if (socketEventReg == NULL) {
        Log_Debug("ERROR: Unable to register socket event: %d (%s)\n", errno, strerror(errno));
        return ExitCode_Init_RegisterIo;
    }
#endif 

    // Create a UART_Config object, open the UART and set up UART event handler
    UART_Config uartConfig;
    UART_InitConfig(&uartConfig);
    uartConfig.baudRate = 115200;
    uartConfig.flowControl = UART_FlowControl_None;
    uartFd = UART_Open(AVNET_MT3620_SK_ISU0_UART, &uartConfig);
    if (uartFd == -1) {
        Log_Debug("ERROR: Could not open UART: %s (%d).\n", strerror(errno), errno);
        return ExitCode_Init_UartOpen;
    }
    uartEventReg = EventLoop_RegisterIo(eventLoop, uartFd, EventLoop_Input, UartEventHandler, NULL);
    if (uartEventReg == NULL) {
        return ExitCode_Init_RegisterIo;
    }

    // Open AVNET_MT3620_SK_USER_BUTTON_A GPIO as input
    Log_Debug("Opening AVNET_MT3620_SK_USER_BUTTON_A as input\n");
    buttonAGpioFd = GPIO_OpenAsInput(AVNET_MT3620_SK_USER_BUTTON_A);
    if (buttonAGpioFd == -1) {
        Log_Debug("ERROR: AVNET_MT3620_SK_USER_BUTTON_A: %s (%d).\n", strerror(errno), errno);
        return ExitCode_Init_MessageButton;
    }

    // Open AVNET_MT3620_SK_USER_BUTTON_A GPIO as input
    Log_Debug("Opening AVNET_MT3620_SK_USER_BUTTON_B as input\n");
    buttonBGpioFd = GPIO_OpenAsInput(AVNET_MT3620_SK_USER_BUTTON_B);
    if (buttonBGpioFd == -1) {
        Log_Debug("ERROR: Could not open AVNET_MT3620_SK_USER_BUTTON_B: %s (%d).\n", strerror(errno), errno);
        return ExitCode_Init_OrientationButton;
    }

    // AVNET_MT3620_SK_APP_STATUS_LED_YELLOW is used to show Device Twin settings state
    Log_Debug("Opening SAMPLE_LED as output\n");
    deviceTwinStatusLedGpioFd =
        GPIO_OpenAsOutput(AVNET_MT3620_SK_APP_STATUS_LED_YELLOW, GPIO_OutputMode_PushPull, GPIO_Value_High);
    if (deviceTwinStatusLedGpioFd == -1) {
        Log_Debug("ERROR: Could not open SAMPLE_LED: %s (%d).\n", strerror(errno), errno);
        return ExitCode_Init_TwinStatusLed;
    }

    // Set up a timer to poll for button events.
    static const struct timespec buttonPressCheckPeriod = {.tv_sec = 0, .tv_nsec = 1000 * 1000};
    buttonPollTimer = CreateEventLoopPeriodicTimer(eventLoop, &ButtonPollTimerEventHandler,
                                                   &buttonPressCheckPeriod);
    if (buttonPollTimer == NULL) {
        return ExitCode_Init_ButtonPollTimer;
    }

    // Set up a timer to interact with Azure
    azureIoTPollPeriodSeconds = AzureIoTDefaultPollPeriodSeconds;
    struct timespec azureTelemetryPeriod = {.tv_sec = azureIoTPollPeriodSeconds, .tv_nsec = 0};
    azureTimer =
        CreateEventLoopPeriodicTimer(eventLoop, &AzureTimerEventHandler, &azureTelemetryPeriod);
    if (azureTimer == NULL) {
        return ExitCode_Init_AzureTimer;
    }

    // Set up a timer to poll the network configuration
    static const struct timespec networkPollPeriod = { .tv_sec = 5, .tv_nsec = 1000 * 0 };
    updateNetworkPollTimer = CreateEventLoopPeriodicTimer(eventLoop, &NetworkPollTimerEventHandler,
        &networkPollPeriod);
    if (updateNetworkPollTimer == NULL) {
        return ExitCode_Init_NetworkPollTimer;
    }

    // Set up a timer to drive the HR4 data collection
    hr4Timer =  CreateEventLoopDisarmedTimer(eventLoop, HR4TimerEventHandler);
            
    // Init the hardware interface to drive the OLED
    init_i2c();
    return ExitCode_Success;
}

/// <summary>
///     Closes a file descriptor and prints an error on failure.
/// </summary>
/// <param name="fd">File descriptor to close</param>
/// <param name="fdName">File descriptor name to use in error message</param>
static void CloseFdAndPrintError(int fd, const char *fdName)
{
    if (fd >= 0) {
        int result = close(fd);
        if (result != 0) {
            Log_Debug("ERROR: Could not close fd %s: %s (%d).\n", fdName, strerror(errno), errno);
        }
    }
}

/// <summary>
///     Close peripherals and handlers.
/// </summary>
static void ClosePeripheralsAndHandlers(void)
{
    DisposeEventLoopTimer(sendTimer);
    EventLoop_UnregisterIo(eventLoop, socketEventReg);
    DisposeEventLoopTimer(buttonPollTimer);
    DisposeEventLoopTimer(azureTimer);
    EventLoop_Close(eventLoop);

    Log_Debug("Closing file descriptors\n");

    // Leave the LEDs off
    if (deviceTwinStatusLedGpioFd >= 0) {
        GPIO_SetValue(deviceTwinStatusLedGpioFd, GPIO_Value_High);
    }
    
    CloseFdAndPrintError(buttonAGpioFd, "ButtonA");
    CloseFdAndPrintError(buttonBGpioFd, "ButtonB");
    CloseFdAndPrintError(deviceTwinStatusLedGpioFd, "StatusLed");
#ifdef M0_INTERCORE_COMMS
    CloseFdAndPrintError(sockFd, "Socket");
#endif 
    CloseFdAndPrintError(i2cFd, "I2C");
}

/// <summary>
///     Sets the IoT Hub authentication state for the app
///     The SAS Token expires which will set the authentication state
/// </summary>
static void HubConnectionStatusCallback(IOTHUB_CLIENT_CONNECTION_STATUS result,
                                        IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason,
                                        void *userContextCallback)
{
    iothubAuthenticated = (result == IOTHUB_CLIENT_CONNECTION_AUTHENTICATED);
    Log_Debug("IoT Hub Authenticated: %s\n", GetReasonString(reason));
}

/// <summary>
///     Sets up the Azure IoT Hub connection (creates the iothubClientHandle)
///     When the SAS Token for a device expires the connection needs to be recreated
///     which is why this is not simply a one time call.
/// </summary>
static void SetupAzureClient(void)
{

    if (iothubClientHandle != NULL) {
        IoTHubDeviceClient_LL_Destroy(iothubClientHandle);
    }

    AZURE_SPHERE_PROV_RETURN_VALUE provResult =
        IoTHubDeviceClient_LL_CreateWithAzureSphereDeviceAuthProvisioning(scopeId, 10000,
                                                                          &iothubClientHandle);
    Log_Debug("IoTHubDeviceClient_LL_CreateWithAzureSphereDeviceAuthProvisioning returned '%s'.\n",
              getAzureSphereProvisioningResultString(provResult));

    if (provResult.result != AZURE_SPHERE_PROV_RESULT_OK) {

        // If we fail to connect, reduce the polling frequency, starting at
        // AzureIoTMinReconnectPeriodSeconds and with a backoff up to
        // AzureIoTMaxReconnectPeriodSeconds
        if (azureIoTPollPeriodSeconds == AzureIoTDefaultPollPeriodSeconds) {
            azureIoTPollPeriodSeconds = AzureIoTMinReconnectPeriodSeconds;
        } else {
            azureIoTPollPeriodSeconds *= 2;
            if (azureIoTPollPeriodSeconds > AzureIoTMaxReconnectPeriodSeconds) {
                azureIoTPollPeriodSeconds = AzureIoTMaxReconnectPeriodSeconds;
            }
        }

        // Reconfigure the timer to enter this routine
        struct timespec azureTelemetryPeriod = {azureIoTPollPeriodSeconds, 0};
        SetEventLoopTimerPeriod(azureTimer, &azureTelemetryPeriod);

        Log_Debug("ERROR: failure to create IoTHub Handle - will retry in %i seconds.\n",
                  azureIoTPollPeriodSeconds);
        return;
    }

    // Successfully connected, so make sure the polling frequency is back to the default
    azureIoTPollPeriodSeconds = AzureIoTDefaultPollPeriodSeconds;
    struct timespec azureTelemetryPeriod = {.tv_sec = azureIoTPollPeriodSeconds, .tv_nsec = 0};
    SetEventLoopTimerPeriod(azureTimer, &azureTelemetryPeriod);

    iothubAuthenticated = true;

    if (IoTHubDeviceClient_LL_SetOption(iothubClientHandle, OPTION_KEEP_ALIVE,
                                        &keepalivePeriodSeconds) != IOTHUB_CLIENT_OK) {
        Log_Debug("ERROR: failure setting option \"%s\"\n", OPTION_KEEP_ALIVE);
        return;
    }

    IoTHubDeviceClient_LL_SetDeviceTwinCallback(iothubClientHandle, TwinCallback, NULL);
    IoTHubDeviceClient_LL_SetConnectionStatusCallback(iothubClientHandle,
                                                      HubConnectionStatusCallback, NULL);
    // Tell the system about the callback function to call when we receive a Direct Method message from Azure
    IoTHubDeviceClient_LL_SetDeviceMethodCallback(iothubClientHandle, DirectMethodCall, NULL);

    // Send up the device information as device twin reported properties
#define ONLINE_MSG_SIZE 500
    static const char cstrOnlineMessage[] = "{ \"manufacturer\":\"Avnet\",\"model\":\"Avnet Azure Sphere Starter Kit\",\"swVersion\":\"%s\", \"osName\":\"Azure Sphere\", \"processorArchitecture\":\"ARM\", \"processorManufacturer\":\"MediaTek\", \"totalMemory\":256,\"totalStorage\": 1024}";
 
    static char str[ONLINE_MSG_SIZE];

    // Construct a JSON telemetry message.
    snprintf(str, ONLINE_MSG_SIZE, cstrOnlineMessage, VERSION_STRING);
    TwinReportString(str);
}

/// <summary>
///     Callback invoked when a Device Twin update is received from IoT Hub.
///     Updates local state for 'showEvents' (bool).
/// </summary>
/// <param name="payload">contains the Device Twin JSON document (desired and reported)</param>
/// <param name="payloadSize">size of the Device Twin JSON document</param>
static void TwinCallback(DEVICE_TWIN_UPDATE_STATE updateState, const unsigned char *payload,
                         size_t payloadSize, void *userContextCallback)
{
    size_t nullTerminatedJsonSize = payloadSize + 1;
    char *nullTerminatedJsonString = (char *)malloc(nullTerminatedJsonSize);
    if (nullTerminatedJsonString == NULL) {
        Log_Debug("ERROR: Could not allocate buffer for twin update payload.\n");
        abort();
    }

    // Copy the provided buffer to a null terminated buffer.
    memcpy(nullTerminatedJsonString, payload, payloadSize);
    // Add the null terminator at the end.
    nullTerminatedJsonString[nullTerminatedJsonSize - 1] = 0;

    JSON_Value *rootProperties = NULL;
    rootProperties = json_parse_string(nullTerminatedJsonString);
    if (rootProperties == NULL) {
        Log_Debug("WARNING: Cannot parse the string as JSON content.\n");
        goto cleanup;
    }

    // Get a pointer to the desired properties object
    JSON_Object *rootObject = json_value_get_object(rootProperties);
    JSON_Object *desiredProperties = json_object_dotget_object(rootObject, "desired");
    if (desiredProperties == NULL) {
        desiredProperties = rootObject;
    }

    // Check to see if the autoRestartHRTest key is in the object.  If so, pull it out and
    // assign the boolean value to the global variable.
    if (json_object_has_value(desiredProperties, "autoRestartHRTest") != 0)
    {
        autoRestartHRTest = (bool)json_object_get_boolean(desiredProperties, "autoRestartHRTest");
        TwinReportBoolState("autoRestartHRTest", autoRestartHRTest);
    }

cleanup:
    // Release the allocated memory.
    json_value_free(rootProperties);
    free(nullTerminatedJsonString);
}

/// <summary>
///     Converts the IoT Hub connection status reason to a string.
/// </summary>
static const char *GetReasonString(IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason)
{
    static char *reasonString = "unknown reason";
    switch (reason) {
    case IOTHUB_CLIENT_CONNECTION_EXPIRED_SAS_TOKEN:
        reasonString = "IOTHUB_CLIENT_CONNECTION_EXPIRED_SAS_TOKEN";
        break;
    case IOTHUB_CLIENT_CONNECTION_DEVICE_DISABLED:
        reasonString = "IOTHUB_CLIENT_CONNECTION_DEVICE_DISABLED";
        break;
    case IOTHUB_CLIENT_CONNECTION_BAD_CREDENTIAL:
        reasonString = "IOTHUB_CLIENT_CONNECTION_BAD_CREDENTIAL";
        break;
    case IOTHUB_CLIENT_CONNECTION_RETRY_EXPIRED:
        reasonString = "IOTHUB_CLIENT_CONNECTION_RETRY_EXPIRED";
        break;
    case IOTHUB_CLIENT_CONNECTION_NO_NETWORK:
        reasonString = "IOTHUB_CLIENT_CONNECTION_NO_NETWORK";
        break;
    case IOTHUB_CLIENT_CONNECTION_COMMUNICATION_ERROR:
        reasonString = "IOTHUB_CLIENT_CONNECTION_COMMUNICATION_ERROR";
        break;
    case IOTHUB_CLIENT_CONNECTION_OK:
        reasonString = "IOTHUB_CLIENT_CONNECTION_OK";
        break;
    }
    return reasonString;
}

/// <summary>
///     Converts AZURE_SPHERE_PROV_RETURN_VALUE to a string.
/// </summary>
static const char *getAzureSphereProvisioningResultString(
    AZURE_SPHERE_PROV_RETURN_VALUE provisioningResult)
{
    switch (provisioningResult.result) {
    case AZURE_SPHERE_PROV_RESULT_OK:
        return "AZURE_SPHERE_PROV_RESULT_OK";
    case AZURE_SPHERE_PROV_RESULT_INVALID_PARAM:
        return "AZURE_SPHERE_PROV_RESULT_INVALID_PARAM";
    case AZURE_SPHERE_PROV_RESULT_NETWORK_NOT_READY:
        return "AZURE_SPHERE_PROV_RESULT_NETWORK_NOT_READY";
    case AZURE_SPHERE_PROV_RESULT_DEVICEAUTH_NOT_READY:
        return "AZURE_SPHERE_PROV_RESULT_DEVICEAUTH_NOT_READY";
    case AZURE_SPHERE_PROV_RESULT_PROV_DEVICE_ERROR:
        return "AZURE_SPHERE_PROV_RESULT_PROV_DEVICE_ERROR";
    case AZURE_SPHERE_PROV_RESULT_GENERIC_ERROR:
        return "AZURE_SPHERE_PROV_RESULT_GENERIC_ERROR";
    default:
        return "UNKNOWN_RETURN_VALUE";
    }
}

/// <summary>
///     Sends simple {"key": "value"} telemetry to IoT Hub
/// </summary>
/// <param name="key">The telemetry item to update</param>
/// <param name="value">new telemetry value</param>
static void SendTelemetry(const unsigned char *key, const unsigned char *value)
{

#if (defined(IOT_CENTRAL_APPLICATION))

    static char eventBuffer[100] = {0};
    static const char *EventMsgTemplate = "{ \"%s\": \"%s\" }";
    int len = snprintf(eventBuffer, sizeof(eventBuffer), EventMsgTemplate, key, value);
    if (len < 0)
        return;

    Log_Debug("Sending IoT Hub Message: %s\n", eventBuffer);

    bool isNetworkingReady = false;
    if ((Networking_IsNetworkingReady(&isNetworkingReady) == -1) || !isNetworkingReady) {
        Log_Debug("WARNING: Cannot send IoTHubMessage because network is not up.\n");
        return;
    }

    IOTHUB_MESSAGE_HANDLE messageHandle = IoTHubMessage_CreateFromString(eventBuffer);

    if (messageHandle == 0) {
        Log_Debug("WARNING: unable to create a new IoTHubMessage\n");
        return;
    }

    if (IoTHubDeviceClient_LL_SendEventAsync(iothubClientHandle, messageHandle, SendMessageCallback,
                                             /*&callback_param*/ 0) != IOTHUB_CLIENT_OK) {
        Log_Debug("WARNING: failed to hand over the message to IoTHubClient\n");
    } else {
        Log_Debug("INFO: IoTHubClient accepted the message for delivery\n");
    }

    IoTHubMessage_Destroy(messageHandle);
#endif 
}

/// <summary>
///     Sends pre-constructed JSON telemetry to IoT Hub
/// </summary>
/// <param name="jsonMessage">A valid JSON string to send as telemetry</param>
void SendJSONTelemetry(const unsigned char* jsonMessage)
{

#ifdef TEST_WITHOUT_SENDING_TELEMETRY
    return;
#endif 
    // Also send this telemetry data to the UART where it will be transmitted over BLE
    // to a mobile device.
    SendUartMessage(uartFd, jsonMessage);

#if (defined(IOT_CENTRAL_APPLICATION))

    Log_Debug("Sending IoT Hub Message: %s\n", jsonMessage);

    bool isNetworkingReady = false;
    if ((Networking_IsNetworkingReady(&isNetworkingReady) == -1) || !isNetworkingReady) {
        Log_Debug("WARNING: Cannot send IoTHubMessage because network is not up.\n");
        return;
    }

    IOTHUB_MESSAGE_HANDLE messageHandle = IoTHubMessage_CreateFromString(jsonMessage);

    if (messageHandle == 0) {
        Log_Debug("WARNING: unable to create a new IoTHubMessage\n");
        return;
    }

    if (IoTHubDeviceClient_LL_SendEventAsync(iothubClientHandle, messageHandle, SendMessageCallback,
        /*&callback_param*/ 0) != IOTHUB_CLIENT_OK) {
        Log_Debug("WARNING: failed to hand over the message to IoTHubClient\n");
    }
    else {
        Log_Debug("INFO: IoTHubClient accepted the message for delivery\n");
    }

    IoTHubMessage_Destroy(messageHandle);
#endif 
}

/// <summary>
///     Callback confirming message delivered to IoT Hub.
/// </summary>
/// <param name="result">Message delivery status</param>
/// <param name="context">User specified context</param>
static void SendMessageCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void *context)
{
    Log_Debug("INFO: Message received by IoT Hub. Result is: %d\n", result);
}

/// <summary>
///     Creates and enqueues a report containing the name and value pair of a Device Twin reported
///     property. The report is not sent immediately, but it is sent on the next invocation of
///     IoTHubDeviceClient_LL_DoWork().
/// </summary>
/// <param name="propertyName">the IoT Hub Device Twin property name</param>
/// <param name="propertyValue">the IoT Hub Device Twin property value</param>
static void TwinReportBoolState(const char *propertyName, bool propertyValue)
{
    if (iothubClientHandle == NULL) {
        Log_Debug("ERROR: client not initialized\n");
    } else {
        static char reportedPropertiesString[30] = {0};
        int len = snprintf(reportedPropertiesString, 30, "{\"%s\":%s}", propertyName,
                           (propertyValue == true ? "true" : "false"));
        if (len < 0)
            return;

        if (IoTHubDeviceClient_LL_SendReportedState(
                iothubClientHandle, (unsigned char *)reportedPropertiesString,
                strlen(reportedPropertiesString), ReportStatusCallback, 0) != IOTHUB_CLIENT_OK) {
            Log_Debug("ERROR: failed to set reported state for '%s'.\n", propertyName);
        } else {
            Log_Debug("INFO: Reported state for '%s' to value '%s'.\n", propertyName,
                      (propertyValue == true ? "true" : "false"));
        }
    }
}

/// <summary>
///     Creates and enqueues a report containing the name and value pair of a Device Twin reported
///     property. The report is not sent immediately, but it is sent on the next invocation of
///     IoTHubDeviceClient_LL_DoWork().
/// </summary>
/// <param name="propertyName">the IoT Hub Device Twin property name</param>
/// <param name="propertyValue">the IoT Hub Device Twin property value</param>
static void TwinReportString(const unsigned char* jsonMessage)
{
    if (iothubClientHandle == NULL) {
        Log_Debug("ERROR: client not initialized\n");
    }
    else {

        if (IoTHubDeviceClient_LL_SendReportedState(
            iothubClientHandle, jsonMessage, 
            strlen(jsonMessage), ReportStatusCallback, 0) != IOTHUB_CLIENT_OK) {
            Log_Debug("ERROR: failed to set reported state for '%s'.\n", jsonMessage);
        }
        else {
            Log_Debug("INFO: Reported state for '%s'.\n", jsonMessage);
        }
    }
}


/// <summary>
///     Callback invoked when the Device Twin reported properties are accepted by IoT Hub.
/// </summary>
static void ReportStatusCallback(int result, void *context)
{
    Log_Debug("INFO: Device Twin reported properties update result: HTTP status code %d\n", result);
}

/// <summary>
///     Check whether a given button has just been pressed.
/// </summary>
/// <param name="fd">The button file descriptor</param>
/// <param name="oldState">Old state of the button (pressed or released)</param>
/// <returns>true if pressed, false otherwise</returns>
static bool IsButtonPressed(int fd, GPIO_Value_Type *oldState)
{
    bool isButtonPressed = false;
    GPIO_Value_Type newState;
    int result = GPIO_GetValue(fd, &newState);
    if (result != 0) {
        Log_Debug("ERROR: Could not read button GPIO: %s (%d).\n", strerror(errno), errno);
        exitCode = ExitCode_IsButtonPressed_GetValue;
    } else {
        // Button is pressed if it is low and different than last known state.
        isButtonPressed = (newState != *oldState) && (newState == GPIO_Value_Low);
        *oldState = newState;
    }

    return isButtonPressed;
}

/// <summary>
/// Pressing SAMPLE_BUTTON_A will: Start or restart a HR test
/// </summary>
static void ButtonAHandler(void)
{
    char *msg;

    if (IsButtonPressed(buttonAGpioFd, &buttonAState)) {
        
        startHR4();
        
        // Set up a timer to poll the network configuration
        static const struct timespec HR4TimerPollPeriod = { .tv_sec = 1, .tv_nsec = 1000 * 0 };
        SetEventLoopTimerPeriod(hr4Timer, &HR4TimerPollPeriod);
    }
}

/// <summary>
/// Pressing SAMPLE_BUTTON_B will:
///     Cycle through the OLED screens
/// </summary>
static void ButtonBHandler(void)
{

    if (IsButtonPressed(buttonBGpioFd, &buttonBState)) {

        Log_Debug("ButtonB\n");

        // Increment to the next OLED screen
        oled_state++;
        if (oled_state > OLED_NUM_SCREEN)
        {
            oled_state = 0;
        }
        update_oled();
    }
}

/// <summary>
///     Handle UART event: if there is incoming data, print it.
///     This satisfies the EventLoopIoCallback signature.
/// </summary>
static void UartEventHandler(EventLoop* el, int fd, EventLoop_IoEvents events, void* context)
{
    const size_t receiveBufferSize = 256;
    uint8_t receiveBuffer[receiveBufferSize + 1]; // allow extra byte for string termination
    ssize_t bytesRead;

    // Read incoming UART data. It is expected behavior that messages may be received in multiple
    // partial chunks.
    bytesRead = read(uartFd, receiveBuffer, receiveBufferSize);
    if (bytesRead == -1) {
        Log_Debug("ERROR: Could not read UART: %s (%d).\n", strerror(errno), errno);
        exitCode = ExitCode_UartEvent_Read;
        return;
    }

    if (bytesRead > 0) {
        // Null terminate the buffer to make it a valid string, and print it
        receiveBuffer[bytesRead] = 0;
        Log_Debug("UART received %d bytes: '%s'.\n", bytesRead, (char*)receiveBuffer);
    }
}

/// <summary>
///     Helper function to send a fixed message via the given UART.
/// </summary>
/// <param name="uartFd">The open file descriptor of the UART to write to</param>
/// <param name="dataToSend">The data to send over the UART</param>
static void SendUartMessage(int uartFd, const char* dataToSend)
{
    size_t totalBytesSent = 0;
    size_t totalBytesToSend = strlen(dataToSend);
    int sendIterations = 0;
    while (totalBytesSent < totalBytesToSend) {
        sendIterations++;

        // Send as much of the remaining data as possible
        size_t bytesLeftToSend = totalBytesToSend - totalBytesSent;
        const char* remainingMessageToSend = dataToSend + totalBytesSent;
        ssize_t bytesSent = write(uartFd, remainingMessageToSend, bytesLeftToSend);
        if (bytesSent == -1) {
            Log_Debug("ERROR: Could not write to UART: %s (%d).\n", strerror(errno), errno);
            exitCode = ExitCode_SendMessage_Write;
            return;
        }

        totalBytesSent += (size_t)bytesSent;
    }
    Log_Debug("Sent %zu bytes over UART in %d calls.\n", totalBytesSent, sendIterations);
}

/// <summary>
///     Direct Method callback function, called when a Direct Method call is received from the Azure
///     IoT Hub.
/// </summary>
/// <param name="methodName">The name of the method being called.</param>
/// <param name="payload">The payload of the method.</param>
/// <param name="responsePayload">The response payload content. This must be a heap-allocated
/// string, 'free' will be called on this buffer by the Azure IoT Hub SDK.</param>
/// <param name="responsePayloadSize">The size of the response payload content.</param>
/// <returns>200 HTTP status code if the method name is reconginized and the payload is correctly parsed;
/// 400 HTTP status code if the payload is invalid;</returns>
/// 404 HTTP status code if the method name is unknown.</returns>
static int DirectMethodCall(const char* methodName, const char* payload, size_t payloadSize, char** responsePayload, size_t* responsePayloadSize)
{
    Log_Debug("\nDirect Method called %s\n", methodName);

    int result = 404; // HTTP status code.

    if (payloadSize < 32) {

        // Declare a char buffer on the stack where we'll operate on a copy of the payload.  
        char directMethodCallContent[payloadSize + 1];

        // Prepare the payload for the response. This is a heap allocated null terminated string.
        // The Azure IoT Hub SDK is responsible of freeing it.
        *responsePayload = NULL;  // Reponse payload content.
        *responsePayloadSize = 0; // Response payload content size.

        // Look for the StartHRTest method name.  This direct method does not require any payload, other than
        // a valid Json argument such as {}.

        if (strcmp(methodName, "StartHRTest") == 0) {

            // Log that the direct method was called and start/restart the HR4 test.
            Log_Debug("StartHRTest() Direct Method called\n");
            startHR4();

            // Construct the response message.  This response will be displayed in the cloud when calling the direct method
            static const char resetOkResponse[] =
                "{ \"success\" : true, \"message\" : \"Starting HR4 Test\" }";
            size_t responseMaxLength = sizeof(resetOkResponse);
            *responsePayload = SetupHeapMessage(resetOkResponse, responseMaxLength);
            if (*responsePayload == NULL) {
                Log_Debug("ERROR: Could not allocate buffer for direct method response payload.\n");
                abort();
            }
            *responsePayloadSize = strlen(*responsePayload);

            result = 200;
            return result;
        }

        else {
            result = 404;
            Log_Debug("INFO: Direct Method called \"%s\" not found.\n", methodName);
            static const char noMethodFound[] = "\"method not found '%s'\"";
            size_t responseMaxLength = sizeof(noMethodFound) + strlen(methodName);
            *responsePayload = SetupHeapMessage(noMethodFound, responseMaxLength, methodName);
            if (*responsePayload == NULL) {
                Log_Debug("ERROR: Could not allocate buffer for direct method response payload.\n");
                abort();
            }
            *responsePayloadSize = strlen(*responsePayload);
            return result;
        }

    }
    else {
        Log_Debug("Payload size > 32 bytes, aborting Direct Method execution\n");
        goto payloadError;
    }

    // If there was a payload error, construct the 
    // response message and send it back to the IoT Hub for the user to see
payloadError:

    result = 400; // Bad request.
    Log_Debug("INFO: Unrecognised direct method payload format.\n");
    static const char noPayloadResponse[] =
        "{ \"success\" : false, \"message\" : \"request does not contain an identifiable "
        "payload\" }";

    size_t responseMaxLength = sizeof(noPayloadResponse) + strlen(payload);
    responseMaxLength = sizeof(noPayloadResponse);
    *responsePayload = SetupHeapMessage(noPayloadResponse, responseMaxLength);
    if (*responsePayload == NULL) {
        Log_Debug("ERROR: Could not allocate buffer for direct method response payload.\n");
        abort();
    }
    *responsePayloadSize = strlen(*responsePayload);

    return result;

}

void startHR4(void) {

    // Initialize the HR4 and pass in the i2c File Descriptor
    hr4_sys_init(i2cFd);

    // Set up a timer to poll the network configuration
    static const struct timespec HR4TimerPollPeriod = { .tv_sec = 1, .tv_nsec = 1000 * 0 };
    SetEventLoopTimerPeriod(hr4Timer, &HR4TimerPollPeriod);

}