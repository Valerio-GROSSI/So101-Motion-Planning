#include "my_so101_robot_hardware_package/tables.hpp"

namespace tables {

const Reg FIRMWARE_MAJOR_VERSION = {0, 1};
const Reg FIRMWARE_MINOR_VERSION = {1, 1};
const Reg MODEL_NUMBER           = {3, 2};

const ControlTable STS_SMS_SERIES_CONTROL_TABLE = {
    // EPROM
    {"Firmware_Major_Version", FIRMWARE_MAJOR_VERSION},
    {"Firmware_Minor_Version", FIRMWARE_MINOR_VERSION},
    {"Model_Number",           MODEL_NUMBER},
    {"ID",                     {5, 1}},
    {"Baud_Rate",              {6, 1}},
    {"Return_Delay_Time",      {7, 1}},
    {"Response_Status_Level",  {8, 1}},
    {"Min_Position_Limit",     {9, 2}},
    {"Max_Position_Limit",     {11, 2}},
    {"Max_Temperature_Limit",  {13, 1}},
    {"Max_Voltage_Limit",      {14, 1}},
    {"Min_Voltage_Limit",      {15, 1}},
    {"Max_Torque_Limit",       {16, 2}},
    {"Phase",                  {18, 1}},
    {"Unloading_Condition",    {19, 1}},
    {"LED_Alarm_Condition",    {20, 1}},
    {"P_Coefficient",          {21, 1}},
    {"D_Coefficient",          {22, 1}},
    {"I_Coefficient",          {23, 1}},
    {"Minimum_Startup_Force",  {24, 2}},
    {"CW_Dead_Zone",           {26, 1}},
    {"CCW_Dead_Zone",          {27, 1}},
    {"Protection_Current",     {28, 2}},
    {"Angular_Resolution",     {30, 1}},
    {"Homing_Offset",          {31, 2}},
    {"Operating_Mode",         {33, 1}},
    {"Protective_Torque",      {34, 1}},
    {"Protection_Time",        {35, 1}},
    {"Overload_Torque",        {36, 1}},
    {"Velocity_closed_loop_P_proportional_coefficient", {37, 1}},
    {"Over_Current_Protection_Time",                    {38, 1}},
    {"Velocity_closed_loop_I_integral_coefficient",     {39, 1}},
    // SRAM
    {"Torque_Enable",          {40, 1}},
    {"Acceleration",           {41, 1}},
    {"Goal_Position",          {42, 2}},
    {"Goal_Time",              {44, 2}},
    {"Goal_Velocity",          {46, 2}},
    {"Torque_Limit",           {48, 2}},
    {"Lock",                   {55, 1}},
    {"Present_Position",       {56, 2}},
    {"Present_Velocity",       {58, 2}},
    {"Present_Load",           {60, 2}},
    {"Present_Voltage",        {62, 1}},
    {"Present_Temperature",    {63, 1}},
    {"Status",                 {65, 1}},
    {"Moving",                 {66, 1}},
    {"Present_Current",        {69, 2}},
    {"Goal_Position_2",        {71, 2}},
    // Factory
    {"Moving_Velocity",        {80, 1}},
    {"Moving_Velocity_Threshold", {80, 1}},
    {"DTs",                    {81, 1}},
    {"Velocity_Unit_factor",   {82, 1}},
    {"Hts",                    {83, 1}},
    {"Maximum_Velocity_Limit", {84, 1}},
    {"Maximum_Acceleration",   {85, 1}},
    {"Acceleration_Multiplier ",{86, 1}},
};

const ControlTable SCS_SERIES_CONTROL_TABLE = {
    // EPROM
    {"Firmware_Major_Version", FIRMWARE_MAJOR_VERSION},
    {"Firmware_Minor_Version", FIRMWARE_MINOR_VERSION},
    {"Model_Number",           MODEL_NUMBER},
    {"ID",                     {5, 1}},
    {"Baud_Rate",              {6, 1}},
    {"Return_Delay_Time",      {7, 1}},
    {"Response_Status_Level",  {8, 1}},
    {"Min_Position_Limit",     {9, 2}},
    {"Max_Position_Limit",     {11, 2}},
    {"Max_Temperature_Limit",  {13, 1}},
    {"Max_Voltage_Limit",      {14, 1}},
    {"Min_Voltage_Limit",      {15, 1}},
    {"Max_Torque_Limit",       {16, 2}},
    {"Phase",                  {18, 1}},
    {"Unloading_Condition",    {19, 1}},
    {"LED_Alarm_Condition",    {20, 1}},
    {"P_Coefficient",          {21, 1}},
    {"D_Coefficient",          {22, 1}},
    {"I_Coefficient",          {23, 1}},
    {"Minimum_Startup_Force",  {24, 2}},
    {"CW_Dead_Zone",           {26, 1}},
    {"CCW_Dead_Zone",          {27, 1}},
    {"Protective_Torque",      {37, 1}},
    {"Protection_Time",        {38, 1}},
    // SRAM
    {"Torque_Enable",          {40, 1}},
    {"Acceleration",           {41, 1}},
    {"Goal_Position",          {42, 2}},
    {"Running_Time",           {44, 2}},
    {"Goal_Velocity",          {46, 2}},
    {"Lock",                   {48, 1}},
    {"Present_Position",       {56, 2}},
    {"Present_Velocity",       {58, 2}},
    {"Present_Load",           {60, 2}},
    {"Present_Voltage",        {62, 1}},
    {"Present_Temperature",    {63, 1}},
    {"Sync_Write_Flag",        {64, 1}},
    {"Status",                 {65, 1}},
    {"Moving",                 {66, 1}},
    // Factory
    {"PWM_Maximum_Step",           {78, 1}},
    {"Moving_Velocity_Threshold*50",{79, 1}},
    {"DTs",                        {80, 1}},
    {"Minimum_Velocity_Limit*50",  {81, 1}},
    {"Maximum_Velocity_Limit*50",  {82, 1}},
    {"Acceleration_2",             {83, 1}},
};

const BaudrateTable STS_SMS_SERIES_BAUDRATE_TABLE = {
    {1'000'000, 0},
    {500'000,   1},
    {250'000,   2},
    {128'000,   3},
    {115'200,   4},
    {57'600,    5},
    {38'400,    6},
    {19'200,    7},
};

const BaudrateTable SCS_SERIES_BAUDRATE_TABLE = STS_SMS_SERIES_BAUDRATE_TABLE;

const EncodingTable EMPTY_ENCODING_TABLE = {};

const EncodingTable STS_SMS_SERIES_ENCODINGS_TABLE = {
    {"Homing_Offset",     11},
    {"Goal_Velocity",     15},
    {"Present_Velocity",  15},
};

const ModelControlTable MODEL_CONTROL_TABLE = {
    {"sts_series", &STS_SMS_SERIES_CONTROL_TABLE},
    {"scs_series", &SCS_SERIES_CONTROL_TABLE},
    {"sms_series", &STS_SMS_SERIES_CONTROL_TABLE},
    {"sts3215",    &STS_SMS_SERIES_CONTROL_TABLE},
    {"sts3250",    &STS_SMS_SERIES_CONTROL_TABLE},
    {"scs0009",    &SCS_SERIES_CONTROL_TABLE},
    {"sm8512bl",   &STS_SMS_SERIES_CONTROL_TABLE},
};

const ModelBaudrateTable MODEL_BAUDRATE_TABLE = {
    {"sts_series", &STS_SMS_SERIES_BAUDRATE_TABLE},
    {"sms_series", &STS_SMS_SERIES_BAUDRATE_TABLE},
    {"scs_series", &SCS_SERIES_BAUDRATE_TABLE},
    {"sm8512bl",   &STS_SMS_SERIES_BAUDRATE_TABLE},
    {"sts3215",    &STS_SMS_SERIES_BAUDRATE_TABLE},
    {"sts3250",    &STS_SMS_SERIES_BAUDRATE_TABLE},
    {"scs0009",    &SCS_SERIES_BAUDRATE_TABLE},
};

const ModelEncodingTable MODEL_ENCODING_TABLE = {
    {"sts_series", &STS_SMS_SERIES_ENCODINGS_TABLE},
    {"sms_series", &STS_SMS_SERIES_ENCODINGS_TABLE},
    {"scs_series", &EMPTY_ENCODING_TABLE},
    {"sts3215",    &STS_SMS_SERIES_ENCODINGS_TABLE},
    {"sts3250",    &STS_SMS_SERIES_ENCODINGS_TABLE},
    {"sm8512bl",   &STS_SMS_SERIES_ENCODINGS_TABLE},
    {"scs0009",    &EMPTY_ENCODING_TABLE},
};

const StringIntMap MODEL_RESOLUTION = {
    {"sts_series", 4096},
    {"sms_series", 4096},
    {"scs_series", 1024},
    {"sts3215",    4096},
    {"sts3250",    4096},
    {"sm8512bl",   65536},
    {"scs0009",    1024},
};

const StringIntMap MODEL_NUMBER_TABLE = {
    {"sts3215", 777},
    {"sts3250", 2825},
    {"sm8512bl", 11272},
    {"scs0009", 1284},
};

const StringIntMap MODEL_PROTOCOL = {
    {"sts_series", 0},
    {"sms_series", 0},
    {"scs_series", 1},
    {"sts3215",    0},
    {"sts3250",    0},
    {"sm8512bl",   0},
    {"scs0009",    1},
};

const std::vector<int> SCAN_BAUDRATES = {
    4'800, 9'600, 14'400, 19'200, 38'400, 57'600, 115'200, 128'000, 250'000, 500'000, 1'000'000
};

} // namespace tables
