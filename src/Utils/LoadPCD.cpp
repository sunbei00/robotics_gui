//
// Created by root on 9/13/24.
//

#include "Utils/LoadPCD.h"
#include <cassert>

namespace Utils{


    bool loadPCD(const std::string& filename, std::map<std::string, std::vector<float>>& points_data) {
        std::ifstream ifs(filename, std::ios::binary);
        if (!ifs) {
            std::cerr << "파일을 열 수 없습니다: " << filename << std::endl;
            return false;
        }

        std::string line;
        std::vector<FieldInfo> fields;
        int width = 0, height = 0, points = 0;
        std::string data_type;
        std::map<std::string, std::string> header;

        // 헤더 읽기
        while (std::getline(ifs, line)) {
            if (line.empty()) continue;
            if (line.substr(0, 1) == "#") {
                // 주석 라인, 건너뜀
                continue;
            } else if (line.substr(0, 4) == "DATA") {
                data_type = line.substr(5);
                break;
            } else {
                // 헤더 라인 파싱
                std::istringstream ss(line);
                std::string key;
                ss >> key;
                std::string value = line.substr(key.length() + 1);
                header[key] = value;
            }
        }

        // FIELDS 파싱
        std::vector<std::string> field_names;
        if (header.find("FIELDS") != header.end()) {
            std::istringstream ss(header["FIELDS"]);
            std::string field;
            while (ss >> field) {
                field_names.push_back(field);
            }
        } else {
            std::cerr << "헤더에 FIELDS가 지정되지 않았습니다." << std::endl;
            return false;
        }

        // SIZE 파싱
        std::vector<int> sizes;
        if (header.find("SIZE") != header.end()) {
            std::istringstream ss(header["SIZE"]);
            int size;
            while (ss >> size) {
                sizes.push_back(size);
            }
        } else {
            std::cerr << "헤더에 SIZE가 지정되지 않았습니다." << std::endl;
            return false;
        }

        // TYPE 파싱
        std::vector<char> types;
        if (header.find("TYPE") != header.end()) {
            std::istringstream ss(header["TYPE"]);
            std::string type_str;
            while (ss >> type_str) {
                if (type_str.length() != 1) {
                    std::cerr << "유효하지 않은 TYPE 값: " << type_str << std::endl;
                    return false;
                }
                types.push_back(type_str[0]);
            }
        } else {
            std::cerr << "헤더에 TYPE이 지정되지 않았습니다." << std::endl;
            return false;
        }

        // COUNT 파싱
        std::vector<int> counts;
        if (header.find("COUNT") != header.end()) {
            std::istringstream ss(header["COUNT"]);
            int count;
            while (ss >> count) {
                counts.push_back(count);
            }
        } else {
            // 기본 COUNT는 1
            counts.resize(field_names.size(), 1);
        }

        // WIDTH 파싱
        if (header.find("WIDTH") != header.end()) {
            width = std::stoi(header["WIDTH"]);
        } else {
            std::cerr << "헤더에 WIDTH가 지정되지 않았습니다." << std::endl;
            return false;
        }

        // HEIGHT 파싱
        if (header.find("HEIGHT") != header.end()) {
            height = std::stoi(header["HEIGHT"]);
        } else {
            std::cerr << "헤더에 HEIGHT가 지정되지 않았습니다." << std::endl;
            return false;
        }

        // POINTS 파싱
        if (header.find("POINTS") != header.end()) {
            points = std::stoi(header["POINTS"]);
        } else {
            points = width * height;
        }

        // FIELDS, SIZE, TYPE, COUNT 길이 검증
        if (field_names.size() != sizes.size() || field_names.size() != types.size() || field_names.size() != counts.size()) {
            std::cerr << "FIELDS, SIZE, TYPE, COUNT의 요소 수가 일치하지 않습니다." << std::endl;
            return false;
        }

        // 필드 정보 구축
        for (size_t i = 0; i < field_names.size(); ++i) {
            FieldInfo field;
            field.name = field_names[i];
            field.size = sizes[i];
            field.type = types[i];
            field.count = counts[i];
            fields.push_back(field);
        }

        // points_data 초기화
        for (const auto& field_name : field_names) {
            points_data[field_name] = std::vector<float>();
        }

        // 각 포인트의 크기 계산
        int point_step = 0;
        for (const auto& field : fields) {
            point_step += field.size * field.count;
        }

        // 데이터 읽기
        if (data_type == "ascii") {
            // ASCII 데이터 읽기
            while (std::getline(ifs, line)) {
                if (line.empty()) continue;
                std::istringstream ss(line);
                for (size_t i = 0; i < fields.size(); ++i) {
                    for (int c = 0; c < fields[i].count; ++c) {
                        std::string value_str;
                        ss >> value_str;
                        if (fields[i].type == 'F') {
                            float value = std::stof(value_str);
                            points_data[fields[i].name].push_back(value);
                        } else if (fields[i].type == 'I' || fields[i].type == 'U') {
                            int value = std::stoi(value_str);
                            points_data[fields[i].name].push_back(static_cast<float>(value));
                        } else {
                            std::cerr << "지원하지 않는 필드 타입: " << fields[i].type << std::endl;
                            return false;
                        }
                    }
                }
            }
        } else if (data_type == "binary") {
            // 바이너리 데이터 읽기
            std::vector<char> data_buffer(point_step * points);
            ifs.read(reinterpret_cast<char*>(data_buffer.data()), data_buffer.size());
            if (ifs.gcount() != static_cast<std::streamsize>(data_buffer.size())) {
                std::cerr << "바이너리 데이터를 읽는 데 실패했습니다." << std::endl;
                return false;
            }

            for (int i = 0; i < points; ++i) {
                int offset = i * point_step;
                for (size_t j = 0; j < fields.size(); ++j) {
                    const FieldInfo& field = fields[j];
                    for (int c = 0; c < field.count; ++c) {
                        if (field.type == 'F') {
                            if (field.size == 4) {
                                float value;
                                memcpy(&value, &data_buffer[offset], sizeof(float));
                                points_data[field.name].push_back(value);
                                offset += sizeof(float);
                            } else if (field.size == 8) {
                                double value;
                                memcpy(&value, &data_buffer[offset], sizeof(double));
                                points_data[field.name].push_back(static_cast<float>(value));
                                offset += sizeof(double);
                            } else {
                                std::cerr << "지원하지 않는 필드 크기: " << field.size << " (필드: " << field.name << ")" << std::endl;
                                return false;
                            }
                        } else if (field.type == 'I') {
                            if (field.size == 1) {
                                int8_t value;
                                memcpy(&value, &data_buffer[offset], sizeof(int8_t));
                                points_data[field.name].push_back(static_cast<float>(value));
                                offset += sizeof(int8_t);
                            } else if (field.size == 2) {
                                int16_t value;
                                memcpy(&value, &data_buffer[offset], sizeof(int16_t));
                                points_data[field.name].push_back(static_cast<float>(value));
                                offset += sizeof(int16_t);
                            } else if (field.size == 4) {
                                int32_t value;
                                memcpy(&value, &data_buffer[offset], sizeof(int32_t));
                                points_data[field.name].push_back(static_cast<float>(value));
                                offset += sizeof(int32_t);
                            } else if (field.size == 8) {
                                int64_t value;
                                memcpy(&value, &data_buffer[offset], sizeof(int64_t));
                                points_data[field.name].push_back(static_cast<float>(value));
                                offset += sizeof(int64_t);
                            } else {
                                std::cerr << "지원하지 않는 필드 크기: " << field.size << " (필드: " << field.name << ")" << std::endl;
                                return false;
                            }
                        } else if (field.type == 'U') {
                            if (field.size == 1) {
                                uint8_t value;
                                memcpy(&value, &data_buffer[offset], sizeof(uint8_t));
                                points_data[field.name].push_back(static_cast<float>(value));
                                offset += sizeof(uint8_t);
                            } else if (field.size == 2) {
                                uint16_t value;
                                memcpy(&value, &data_buffer[offset], sizeof(uint16_t));
                                points_data[field.name].push_back(static_cast<float>(value));
                                offset += sizeof(uint16_t);
                            } else if (field.size == 4) {
                                uint32_t value;
                                memcpy(&value, &data_buffer[offset], sizeof(uint32_t));
                                points_data[field.name].push_back(static_cast<float>(value));
                                offset += sizeof(uint32_t);
                            } else if (field.size == 8) {
                                uint64_t value;
                                memcpy(&value, &data_buffer[offset], sizeof(uint64_t));
                                points_data[field.name].push_back(static_cast<float>(value));
                                offset += sizeof(uint64_t);
                            } else {
                                std::cerr << "지원하지 않는 필드 크기: " << field.size << " (필드: " << field.name << ")" << std::endl;
                                return false;
                            }
                        } else {
                            std::cerr << "지원하지 않는 필드 타입: " << field.type << std::endl;
                            return false;
                        }
                    }
                }
            }
        } else {
            std::cerr << "지원하지 않는 DATA 타입: " << data_type << std::endl;
            return false;
        }

        ifs.close();
        assert(points_data["x"].size() == points_data["y"].size() &&  points_data["y"].size()  == points_data["z"].size());
        std::cout << "성공적으로 " << points << "개의 포인트를 " << filename << "에서 불러왔습니다." << std::endl;
        return true;
    }

}