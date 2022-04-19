#include "Pipe.h"

GocatorCV::Pipe::Pipe() {
	// constructor
}

void GocatorCV::Pipe::SendPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int id) {
	auto start = std::chrono::high_resolution_clock::now();
	
	nlohmann::json j = { };

	j["point_cloud"]["id"] = id;
	/*
	for (int i = 0; i < pcl.x.size(); i++) {
		j["point_cloud"]["points"][i]["x"] = pcl.x[i];
		j["point_cloud"]["points"][i]["y"] = pcl.y[i];
		j["point_cloud"]["points"][i]["z"] = pcl.z[i];
	}
	*/
	for (int i = 0; i < cloud->size(); i++) {
		j["point_cloud"]["points"][i]["x"] = cloud->points[i].x;
		j["point_cloud"]["points"][i]["y"] = cloud->points[i].y;
		j["point_cloud"]["points"][i]["z"] = cloud->points[i].z;
	}

	std::string result = j.dump();
	
	std::string _result = result + "\r\n";

	// create file
	HANDLE fileHandle = CreateFileW(TEXT("\\\\.\\pipe\\gocator-pipe"), GENERIC_READ | GENERIC_WRITE, FILE_SHARE_WRITE, NULL, OPEN_EXISTING, 0, NULL);
	DWORD dwWritten;

	// send data to server
	if (fileHandle != INVALID_HANDLE_VALUE) {
		std::cout << "send from pipe server: " << strlen((const char*)_result.c_str()) << " bytes" << std::endl;
		WriteFile(fileHandle, (const char*)_result.c_str(), strlen((const char*)_result.c_str()), &dwWritten, NULL);

		CloseHandle(fileHandle);
	}

	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
	std::cout << "Time taken by function SendPCL: " << duration.count() << " microseconds" << std::endl;
}

void GocatorCV::Pipe::SendStats(GocatorCV::Statistics stats, int id) {
	auto start = std::chrono::high_resolution_clock::now();
	
	nlohmann::json j = {};

	j["stats"]["id"] = id;

	std::cout << "row size: " << stats.row.size() << std::endl;
	for (int i = 0; i < stats.row.size(); i++) {
		j["stats"]["rows"][i]["row"] = stats.row[i];
	}

	std::string result = j.dump();

	std::string _result = result + "\r\n";

	// create file
	HANDLE fileHandle = CreateFileW(TEXT("\\\\.\\pipe\\gocator-pipe"), GENERIC_READ | GENERIC_WRITE, FILE_SHARE_WRITE, NULL, OPEN_EXISTING, 0, NULL);
	DWORD dwWritten;

	// send data to server
	if (fileHandle != INVALID_HANDLE_VALUE) {
		WriteFile(fileHandle, (const char*)_result.c_str(), strlen((const char*)_result.c_str()), &dwWritten, NULL);
		std::cout << "send from pipe server: " << strlen((const char*)_result.c_str()) << " bytes" << std::endl;

		CloseHandle(fileHandle);
	}
	
	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
	std::cout << "Time taken by function SendStats: " << duration.count() << " microseconds" << std::endl;
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