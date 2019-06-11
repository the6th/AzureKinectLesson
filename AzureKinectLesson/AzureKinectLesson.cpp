// AzureKinectLesson.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <math.h>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdio.h>
#include <malloc.h>
#include <k4a/k4a.h>
#include <k4arecord/playback.h>

typedef struct
{
	char *filename;
	k4a_playback_t handle;
	k4a_record_configuration_t record_config;
	k4a_capture_t capture;
} recording_t;

static uint64_t first_capture_timestamp(k4a_capture_t capture)
{
	uint64_t min_timestamp = (uint64_t)-1;
	k4a_image_t images[3];
	images[0] = k4a_capture_get_color_image(capture);
	images[1] = k4a_capture_get_depth_image(capture);
	images[2] = k4a_capture_get_ir_image(capture);

	for (int i = 0; i < 3; i++)
	{
		if (images[i] != NULL)
		{
			//uint64_t timestamp = k4a_image_get_device_timestamp_usec(images[i]);
			//if (timestamp < min_timestamp)
			//{
			//	min_timestamp = timestamp;
			//}

			
			k4a_image_release(images[i]);
			images[i] = NULL;
		}
	}

	return min_timestamp;
}

static void print_capture_info(recording_t *file)
{
	k4a_image_t images[3];
	images[0] = k4a_capture_get_color_image(file->capture);
	images[1] = k4a_capture_get_depth_image(file->capture);
	images[2] = k4a_capture_get_ir_image(file->capture);

	printf("%-32s", file->filename);
	for (int i = 0; i < 3; i++)
	{
		if (images[i] != NULL)
		{
			//uint64_t timestamp = k4a_image_get_device_timestamp_usec(images[i]) +
			//	(uint64_t)file->record_config.start_timestamp_offset_usec;
			//printf("  %7ju usec", timestamp);
			k4a_image_release(images[i]);
			images[i] = NULL;
		}
		else
		{
			printf("  %12s", "");
		}
	}
	printf("\n");
}


static void create_xy_table(const k4a_calibration_t *calibration, k4a_image_t xy_table)
{
	k4a_float2_t *table_data = (k4a_float2_t *)(void *)k4a_image_get_buffer(xy_table);

	int width = calibration->depth_camera_calibration.resolution_width;
	int height = calibration->depth_camera_calibration.resolution_height;

	k4a_float2_t p;
	k4a_float3_t ray;
	int valid;

	for (int y = 0, idx = 0; y < height; y++)
	{
		p.xy.y = (float)y;
		for (int x = 0; x < width; x++, idx++)
		{
			p.xy.x = (float)x;

			k4a_calibration_2d_to_3d(
				calibration, &p, 1.f, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &ray, &valid);

			if (valid)
			{
				table_data[idx].xy.x = ray.xyz.x;
				table_data[idx].xy.y = ray.xyz.y;
			}
			else
			{
				table_data[idx].xy.x = nanf("");
				table_data[idx].xy.y = nanf("");
			}
		}
	}
}

static void generate_point_cloud(const k4a_image_t depth_image,
	const k4a_image_t xy_table,
	k4a_image_t point_cloud,
	int *point_count)
{
	int width = k4a_image_get_width_pixels(depth_image);
	int height = k4a_image_get_height_pixels(depth_image);

	uint16_t *depth_data = (uint16_t *)(void *)k4a_image_get_buffer(depth_image);
	k4a_float2_t *xy_table_data = (k4a_float2_t *)(void *)k4a_image_get_buffer(xy_table);
	k4a_float3_t *point_cloud_data = (k4a_float3_t *)(void *)k4a_image_get_buffer(point_cloud);

	*point_count = 0;
	for (int i = 0; i < width * height; i++)
	{
		if (depth_data[i] != 0 && !isnan(xy_table_data[i].xy.x) && !isnan(xy_table_data[i].xy.y))
		{
			point_cloud_data[i].xyz.x = xy_table_data[i].xy.x * (float)depth_data[i];
			point_cloud_data[i].xyz.y = xy_table_data[i].xy.y * (float)depth_data[i];
			point_cloud_data[i].xyz.z = (float)depth_data[i];
			(*point_count)++;
		}
		else
		{
			point_cloud_data[i].xyz.x = nanf("");
			point_cloud_data[i].xyz.y = nanf("");
			point_cloud_data[i].xyz.z = nanf("");
		}
	}
}

static void write_point_cloud(const char *file_name, const k4a_image_t point_cloud, int point_count)
{
	int width = k4a_image_get_width_pixels(point_cloud);
	int height = k4a_image_get_height_pixels(point_cloud);

	k4a_float3_t *point_cloud_data = (k4a_float3_t *)(void *)k4a_image_get_buffer(point_cloud);

	// save to the ply file
	std::ofstream ofs(file_name); // text mode first
	ofs << "ply" << std::endl;
	ofs << "format ascii 1.0" << std::endl;
	ofs << "element vertex"
		<< " " << point_count << std::endl;
	ofs << "property float x" << std::endl;
	ofs << "property float y" << std::endl;
	ofs << "property float z" << std::endl;
	ofs << "end_header" << std::endl;
	ofs.close();

	std::stringstream ss;
	for (int i = 0; i < width * height; i++)
	{
		if (isnan(point_cloud_data[i].xyz.x) || isnan(point_cloud_data[i].xyz.y) || isnan(point_cloud_data[i].xyz.z))
		{
			continue;
		}

		ss << (float)point_cloud_data[i].xyz.x << " " << (float)point_cloud_data[i].xyz.y << " "
			<< (float)point_cloud_data[i].xyz.z << std::endl;
	}

	std::ofstream ofs_text(file_name, std::ios::out | std::ios::app);
	ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}


int main1(int argc, char **argv)
{

	std::cout << "Hello World!\n";


	if (argc != 2)
	{
		printf("Usage: playback_external_sync.exe <file.mkv>\n");
		std::cout << "Hello World 2\n";
		return 1;
	}



	//bool master_found = false;
	k4a_result_t result = K4A_RESULT_SUCCEEDED;

	// Allocate memory to store the state of N recordings.
	recording_t *files = (recording_t *)malloc(sizeof(recording_t));
	std::cout << "Hello World 3\n";

	if (files == NULL)
	{
		printf("Failed to allocate memory for playback (%zu bytes)\n", sizeof(recording_t));
		return 1;
	}
	else {
		printf("generate to allocate memory for playback (%zu bytes)\n", sizeof(recording_t));
	}

	memset(files, 0, sizeof(recording_t));

	int i = 0;

	files[i].filename = argv[i + 1];
	result = k4a_playback_open(files[i].filename, &files[i].handle);
	if (result != K4A_RESULT_SUCCEEDED)
	{
		printf("Failed to open file: %s\n", files[i].filename);
	}

	result = k4a_playback_get_record_configuration(files[i].handle, &files[i].record_config);
	if (result != K4A_RESULT_SUCCEEDED)
	{
		printf("Failed to get record configuration for file: %s\n", files[i].filename);
	}

	//if (files[i].record_config.wired_sync_mode == K4A_WIRED_SYNC_MODE_MASTER)

	
	if (files[i].record_config.wired_sync_mode == K4A_WIRED_SYNC_MODE_STANDALONE)
	{
		printf("Opened standalone recording file: %s\n", files[i].filename);
		
	}
	else
	{
		printf("ERROR: Recording file was not recorded in Standalone mode: %s\n", files[i].filename);
		result = K4A_RESULT_FAILED;
	}

	// Read the first capture of each recording into memory.
	k4a_stream_result_t stream_result = k4a_playback_get_next_capture(files[i].handle, &files[i].capture);
	if (stream_result == K4A_STREAM_RESULT_EOF)
	{
		printf("ERROR: Recording file is empty: %s\n", files[i].filename);
		result = K4A_RESULT_FAILED;
	}
	else if (stream_result == K4A_STREAM_RESULT_FAILED)
	{
		printf("ERROR: Failed to read first capture from file: %s\n", files[i].filename);
		result = K4A_RESULT_FAILED;
	}



	if (result == K4A_RESULT_SUCCEEDED)
	{
		printf("%-32s  %12s  %12s  %12s\n", "Source file", "COLOR", "DEPTH", "IR");
		printf("==========================================================================\n");

		// Print the first 25 captures in order of timestamp across all the recordings.
		for (int frame = 0; frame < 25; frame++)
		{
			uint64_t min_timestamp = (uint64_t)-1;
			recording_t *min_file = NULL;

			if (files[i].capture != NULL)
			{
				// All recording files start at timestamp 0, however the first timestamp off the camera is usually
				// non-zero. We need to add the recording "start offset" back to the recording timestamp to recover
				// the original timestamp from the device, and synchronize the files.
				uint64_t timestamp = first_capture_timestamp(files[i].capture) +
					files[i].record_config.start_timestamp_offset_usec;
				if (timestamp < min_timestamp)
				{
					min_timestamp = timestamp;
					min_file = &files[i];
				}
			}


			print_capture_info(min_file);

			k4a_capture_release(min_file->capture);
			min_file->capture = NULL;

			// Advance the recording with the lowest current timestamp forward.
			k4a_stream_result_t stream_result = k4a_playback_get_next_capture(min_file->handle, &min_file->capture);
			if (stream_result == K4A_STREAM_RESULT_FAILED)
			{
				printf("ERROR: Failed to read next capture from file: %s\n", min_file->filename);
				result = K4A_RESULT_FAILED;
				break;
			}
		}
	}

	if (files[i].handle != NULL)
	{
		k4a_playback_close(files[i].handle);
		files[i].handle = NULL;
	}

	free(files);
	return result == K4A_RESULT_SUCCEEDED ? 0 : 1;
}



int main2(int argc, char **argv)
{
	int returnCode = 1;
	k4a_device_t device = NULL;
	const int32_t TIMEOUT_IN_MS = 1000;
	k4a_capture_t capture = NULL;
	std::string file_name;
	uint32_t device_count = 0;
	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	k4a_image_t depth_image = NULL;
	k4a_image_t xy_table = NULL;
	k4a_image_t point_cloud = NULL;
	int point_count = 0;

	if (argc != 2)
	{
		printf("fastpointcloud.exe <output file>\n");
		returnCode = 2;
		goto Exit;
	}

	file_name = argv[1];

	device_count = k4a_device_get_installed_count();

	if (device_count == 0)
	{
		printf("No K4A devices found\n");
		return 0;
	}

	if (K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &device))
	{
		printf("Failed to open device\n");
		goto Exit;
	}

	config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
	config.camera_fps = K4A_FRAMES_PER_SECOND_30;

	k4a_calibration_t calibration;
	if (K4A_RESULT_SUCCEEDED !=
		k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration))
	{
		printf("Failed to get calibration\n");
		goto Exit;
	}

	k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
		calibration.depth_camera_calibration.resolution_width,
		calibration.depth_camera_calibration.resolution_height,
		calibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float2_t),
		&xy_table);

	create_xy_table(&calibration, xy_table);

	k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
		calibration.depth_camera_calibration.resolution_width,
		calibration.depth_camera_calibration.resolution_height,
		calibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float3_t),
		&point_cloud);

	if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config))
	{
		printf("Failed to start cameras\n");
		goto Exit;
	}

	// Get a capture
	switch (k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS))
	{
	case K4A_WAIT_RESULT_SUCCEEDED:
		break;
	case K4A_WAIT_RESULT_TIMEOUT:
		printf("Timed out waiting for a capture\n");
		goto Exit;
	case K4A_WAIT_RESULT_FAILED:
		printf("Failed to read a capture\n");
		goto Exit;
	}

	// Get a depth image
	depth_image = k4a_capture_get_depth_image(capture);
	if (depth_image == 0)
	{
		printf("Failed to get depth image from capture\n");
		goto Exit;
	}

	generate_point_cloud(depth_image, xy_table, point_cloud, &point_count);

	write_point_cloud(file_name.c_str(), point_cloud, point_count);

	k4a_image_release(depth_image);
	k4a_capture_release(capture);
	k4a_image_release(xy_table);
	k4a_image_release(point_cloud);

	returnCode = 0;
Exit:
	if (device != NULL)
	{
		k4a_device_close(device);
	}

	return returnCode;
}

int main(int argc, char **argv) {
	main1(argc, argv);
	//main2(argc, argv);

}



// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
