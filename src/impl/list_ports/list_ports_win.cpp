/* 
 * Copyright (c) 2023 Guillaume Guillet
 * Original from :
 *   Copyright (c) 2012 William Woodall, John Harrison, Craig Lilley
 *   https://github.com/wjwwood/serial
 */

#ifdef _WIN32

#include "serial/serial.hpp"
#include <tchar.h>

#define WIN32_LEAN_AND_MEAN
#ifndef NOMINMAX
    #define NOMINMAX
#endif
#include <windows.h>

#include <setupapi.h>
#include <initguid.h>
#include <devguid.h>

constexpr DWORD port_name_max_length = 256;
constexpr DWORD friendly_name_max_length = 256;
constexpr DWORD hardware_id_max_length = 256;

// Convert a wide Unicode string to an UTF8 string
std::string utf8_encode(const std::wstring &wstr)
{
	int size_needed = WideCharToMultiByte(CP_UTF8, 0, wstr.data(), static_cast<int>(wstr.size()), 
                                          NULL, 0, NULL, NULL);
	std::string strTo(size_needed, 0);
	WideCharToMultiByte(CP_UTF8, 0, wstr.data(), static_cast<int>(wstr.size()), 
                        &strTo[0], size_needed, NULL, NULL);
	return strTo;
}

std::vector<serial::PortInfo> serial::list_ports()
{
	std::vector<PortInfo> devices_found;

	HDEVINFO device_info_set = SetupDiGetClassDevs(
            static_cast<const GUID *>(&GUID_DEVCLASS_PORTS),
            NULL,
            NULL,
            DIGCF_PRESENT);

	unsigned int device_info_set_index = 0;
	SP_DEVINFO_DATA device_info_data;

	device_info_data.cbSize = sizeof(SP_DEVINFO_DATA);

	while (SetupDiEnumDeviceInfo(device_info_set, device_info_set_index, &device_info_data))
	{
		device_info_set_index++;

		// Get port name

		HKEY hkey = SetupDiOpenDevRegKey(
                device_info_set,
                &device_info_data,
                DICS_FLAG_GLOBAL,
                0,
                DIREG_DEV,
                KEY_READ);

		TCHAR port_name[port_name_max_length];
		DWORD port_name_length = port_name_max_length;

		LONG return_code = RegQueryValueEx(
                hkey,
                _T("PortName"),
                NULL,
                NULL,
                reinterpret_cast<LPBYTE>(port_name),
                &port_name_length);

		RegCloseKey(hkey);

		if (return_code != EXIT_SUCCESS)
        {
            continue;
        }

		if(port_name_length > 0 && port_name_length <= port_name_max_length)
        {
            port_name[port_name_length - 1] = '\0';
        }
		else
        {
            port_name[0] = '\0';
        }

		// Ignore parallel ports
		if(_tcsstr(port_name, _T("LPT")) != NULL)
        {
            continue;
        }

		// Get port friendly name

		TCHAR friendly_name[friendly_name_max_length];
		DWORD friendly_name_actual_length = 0;

		BOOL got_friendly_name = SetupDiGetDeviceRegistryProperty(
					device_info_set,
					&device_info_data,
					SPDRP_FRIENDLYNAME,
					NULL,
					(PBYTE)friendly_name,
					friendly_name_max_length,
					&friendly_name_actual_length);

		if(got_friendly_name == TRUE && friendly_name_actual_length > 0)
        {
            friendly_name[friendly_name_actual_length - 1] = '\0';
        }
		else
        {
            friendly_name[0] = '\0';
        }

		// Get hardware ID

		TCHAR hardware_id[hardware_id_max_length];
		DWORD hardware_id_actual_length = 0;

		BOOL got_hardware_id = SetupDiGetDeviceRegistryProperty(
					device_info_set,
					&device_info_data,
					SPDRP_HARDWAREID,
					NULL,
					(PBYTE)hardware_id,
					hardware_id_max_length,
					&hardware_id_actual_length);

		if(got_hardware_id == TRUE && hardware_id_actual_length > 0)
        {
            hardware_id[hardware_id_actual_length - 1] = '\0';
        }
		else
        {
            hardware_id[0] = '\0';
        }

		#ifdef UNICODE
			std::string portName = utf8_encode(port_name);
			std::string friendlyName = utf8_encode(friendly_name);
			std::string hardwareId = utf8_encode(hardware_id);
		#else
			std::string portName = port_name;
			std::string friendlyName = friendly_name;
			std::string hardwareId = hardware_id;
		#endif

		PortInfo port_entry;
		port_entry.port = portName;
		port_entry.description = friendlyName;
		port_entry.hardware_id = hardwareId;

		devices_found.push_back(port_entry);
	}

	SetupDiDestroyDeviceInfoList(device_info_set);

	return devices_found;
}

#endif // #ifdef _WIN32
