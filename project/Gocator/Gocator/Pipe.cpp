#include "Pipe.h"

GocatorCV::Pipe::Pipe() {
	// constructor
}

void GocatorCV::Pipe::SendPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int id, int n) {
	
	auto start = std::chrono::high_resolution_clock::now();
	
	nlohmann::json j = { };

	j["point_cloud"]["id"] = id;
	j["point_cloud"]["n"] = n;

	for (int i = 0; i < cloud->size(); i++) {
		j["point_cloud"]["points"][i]["x"] = cloud->points[i].x;
		j["point_cloud"]["points"][i]["y"] = cloud->points[i].y;
		j["point_cloud"]["points"][i]["z"] = cloud->points[i].z;
	}

	std::string result = j.dump();
	std::string _result = result + "\r\n";

	// create file
	DWORD last_error;
	DWORD numBytesWritten = 0;
	HANDLE pipe;
	unsigned int elapsed_seconds = 0;
	const unsigned int timeout_seconds = 5;

	pipe = CreateFileW(TEXT("\\\\.\\pipe\\gocator-pipe"), GENERIC_READ | GENERIC_WRITE, FILE_SHARE_WRITE, NULL, OPEN_EXISTING, 0, NULL);

	while (pipe == INVALID_HANDLE_VALUE && elapsed_seconds < timeout_seconds) {
		last_error = GetLastError();
		Sleep(100);
		elapsed_seconds = elapsed_seconds + 0.1;

		pipe = CreateFileW(TEXT("\\\\.\\pipe\\gocator-pipe"), GENERIC_READ | GENERIC_WRITE, FILE_SHARE_WRITE, NULL, OPEN_EXISTING, 0, NULL);
	}

	if (pipe == INVALID_HANDLE_VALUE) {
		std::cerr << "Failed to connect to pipe: last_error=" << last_error << "\n";
	}
	
	// send data to server
	if (pipe != INVALID_HANDLE_VALUE) {
		std::cout << "Number of bytes sent: " << strlen((const char*)_result.c_str()) << " bytes" << std::endl;
		WriteFile(pipe, (const char*)_result.c_str(), strlen((const char*)_result.c_str()), &numBytesWritten, NULL);

		CloseHandle(pipe);
	}

	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
	std::cout << "Time taken by function SendPCL: " << duration.count() << " milliseconds" << std::endl;
}

void GocatorCV::Pipe::SendStats(GocatorCV::Statistics stats, int id) {
	
	auto start = std::chrono::high_resolution_clock::now();
	
	nlohmann::json j = {};

	j["stats"]["id"] = id;

	for (int i = 0; i < stats.row.size(); i++) {
		j["stats"]["rows"][i]["row"] = stats.row[i];
	}

	std::string result = j.dump();
	std::string _result = result + "\r\n";

	// create file
	DWORD last_error;
	DWORD numBytesWritten = 0;
	HANDLE pipe;
	unsigned int elapsed_seconds = 0;
	const unsigned int timeout_seconds = 5;

	pipe = CreateFileW(TEXT("\\\\.\\pipe\\gocator-pipe"), GENERIC_READ | GENERIC_WRITE, FILE_SHARE_WRITE, NULL, OPEN_EXISTING, 0, NULL);

	while (pipe == INVALID_HANDLE_VALUE && elapsed_seconds < timeout_seconds) {
		last_error = GetLastError();
		Sleep(100);
		elapsed_seconds = elapsed_seconds + 0.1;

		pipe = CreateFileW(TEXT("\\\\.\\pipe\\gocator-pipe"), GENERIC_READ | GENERIC_WRITE, FILE_SHARE_WRITE, NULL, OPEN_EXISTING, 0, NULL);
	}

	if (pipe == INVALID_HANDLE_VALUE) {
		std::cerr << "Failed to connect to pipe: last_error=" << last_error << "\n";
	}

	// send data to server
	if (pipe != INVALID_HANDLE_VALUE) {
		WriteFile(pipe, (const char*)_result.c_str(), strlen((const char*)_result.c_str()), &numBytesWritten, NULL);
		std::cout << "Number of bytes sent: " << strlen((const char*)_result.c_str()) << " bytes" << std::endl;

		CloseHandle(pipe);
	}
	
	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
	std::cout << "Time taken by function SendStats: " << duration.count() << " milliseconds" << std::endl;
}

std::string GocatorCV::Pipe::string_to_hex(const std::string& input) {
	static const char hex_digits[] = "0123456789ABCDEF";

	std::string output;
	output.reserve(input.length() * 2);
	for (unsigned char c : input) {
		output.push_back(hex_digits[c >> 4]);
		output.push_back(hex_digits[c & 15]);
	}
	return output;
}