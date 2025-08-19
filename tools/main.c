#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/errno.h>
#include <stdint.h>
#include <stdbool.h>

typedef float float32_t;

typedef struct {
	uint32_t num;
	float32_t kp;
	float32_t ki;
	float32_t kd;
} joint_params_t;

typedef struct {
	char num[10U];
	char kp[10U];
	char ki[10U];
	char kd[10U];
} joint_strings_t;

bool joint_strings_to_params(joint_strings_t const* strings, joint_params_t* params)
{
	if (strings == NULL || params == NULL) {
		return false;
	}

	if (strings->num == NULL || strings->kp == NULL || 
		strings->ki == NULL || strings->kd == NULL) {
		return false;
	}

	sscanf(strings->num, "%u", &params->num);
	sscanf(strings->kp, "%f", &params->kp);
	sscanf(strings->ki, "%f", &params->ki);
	sscanf(strings->kd, "%f", &params->kd);

	return true;
}

void replace_whitespace_with_zeros(char* string, size_t length) 
{
	if (string == NULL || length == 0UL) {
		return;
	}

	char* string_end = string + length;
	for (char* it = string; it != string_end; ++it) {
		if (*it == ' ' || *it == '\t') {
			*it = '\0';
		}
	}
}

bool load_joint_strings_from_file(char const* file_name, joint_strings_t* strings)
{
	if (file_name == NULL || strings == NULL) {
		return false;
	} 

	struct stat file_status;
	if (stat(file_name, &file_status) != 0) {
		fprintf(stderr, "Failed stating file: %s!\n", file_name);
		return false;
	}
	

	FILE* file = fopen(file_name, "r");
	if (file == NULL) {
		fprintf(stderr, "Failed opening file: %s!\n", file_name);
		return false;
	}

	char line[100U];
	while (fgets(line, sizeof(line), file) != NULL) {
		char* sep = strchr(line, ':');
		if (sep == NULL) {
			continue;
		}
		
		char* key = line;
		*sep = '\0';
		char* value = sep + 1U;


		// replace_whitespace_with_zeros(key, value - key);
		// replace_whitespace_with_zeros(value, line + sizeof(line) - value);
		
		if (strncmp(key, "num", sizeof(key)) == 0) {
			strncpy(strings->num, value , sizeof(strings->num));
		} else if (strncmp(key, "kp", sizeof(key)) == 0) {
			strncpy(strings->kp, value, sizeof(strings->kp));
		} else if (strncmp(key, "ki", sizeof(key)) == 0) {
			strncpy(strings->ki, value, sizeof(strings->ki));
		} else if (strncmp(key, "kd", sizeof(key)) == 0) {
			strncpy(strings->kd, value, sizeof(strings->kd));
		}
	}

	return fclose(file) == 0;
}

bool save_joint_params_to_file(char const* file_name, joint_params_t const* params)
{
	if (file_name == NULL || params == NULL) {
		return false;	
	}

	struct stat file_status;
	if (stat(file_name, &file_status) != 0) {
		fprintf(stderr, "Failed stating file: %s!\n", file_name);
		return false;
	}

	FILE* file = fopen(file_name, "w");
	if (file == NULL) {
		fprintf(stderr, "Failed opening file: %s\n!", file_name);
		return false;
	}

	char line[100U];
	snprintf(line, sizeof(line), "{.num = %u,\n .kp = %f,\n .ki = %f,\n .kd = %f}", 
			 				params->num, params->kp, params->ki, params->kd);
	fputs(line, file);

	return fclose(file) == 0;
}

int main(int argc, char const** argv)
{
	if (argc != 3) {
		fprintf(stderr, "Usage: %s <src_filename> <dest_filename>\n", argv[0]);
		return 1;
	}

	joint_strings_t strings;	
	if (!load_joint_strings_from_file(argv[1], &strings)) {
		fprintf(stderr, "failed loading from file!\n");
		return -1;
	}

	joint_params_t params;
	joint_strings_to_params(&strings, &params);

	if (!save_joint_params_to_file(argv[2], &params)) {
		fprintf(stderr, "failed saving to file!\n");
		return -1;
	}

	fprintf(stdout, "num: %s, kp: %s, ki: %s, kd: %s\n", strings.num, strings.kp, strings.ki, strings.kd);	

	return 0;
}

