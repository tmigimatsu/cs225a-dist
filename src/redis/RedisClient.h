/**
 * RedisClient.h
 *
 * Author: Toki Migimatsu
 * Created: April 2017
 */

#ifndef REDIS_CLIENT_H
#define REDIS_CLIENT_H

#include <Eigen/Core>
#include <hiredis/hiredis.h>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <stdexcept>

// #define JSON_DEFAULT

namespace RedisServer {
	// Default server ip
	const std::string DEFAULT_IP = "127.0.0.1";

	// Default server port
	const int DEFAULT_PORT = 6379;
}

struct redisReplyDeleter {
	void operator()(redisReply *r) { freeReplyObject(r); }
};
struct redisContextDeleter {
	void operator()(redisContext *c) { redisFree(c); }
};

class RedisClient {

public:
	std::unique_ptr<redisContext, redisContextDeleter> context_;

	/**
 	 * Connect to Redis server.
 	 *
 	 * @param hostname  Redis server IP address (default 127.0.0.1).
 	 * @param port      Redis server port number (default 6379).
 	 * @param timeout   Connection attempt timeout (default 1.5s).
 	 */
	void connect(const std::string& hostname="127.0.0.1", const int port=6379,
	             const struct timeval& timeout={1, 500000});

	/**
 	 * Issue a command to Redis.
 	 *
 	 * This function is a C++ wrapper around hiredis::redisCommand() that
 	 * provides a self-freeing redisReply pointer. The command is formatted in
 	 * printf() style.
 	 *
 	 * @param format  Format string.
 	 * @param ...     Format values.
 	 * @return        redisReply pointer.
 	 */
	std::unique_ptr<redisReply, redisReplyDeleter> command(const char *format, ...);

	/**
 	 * Perform Redis command: PING.
 	 *
 	 * If the server is responsive, it should reply PONG.
 	 */
	void ping();

	/**
	 * Perform Redis command: GET key.
	 *
	 * @param key  Key to get from Redis (entry must be String type).
	 * @return     String value.
	 */
	std::string get(const std::string& key);

	/**
	 * Perform Redis command: SET key value.
	 *
	 * @param key    Key to set in Redis.
	 * @param value  Value for key.
	 */
	void set(const std::string& key, const std::string& value);

	/**
	 * Perform Redis command: DEL key.
	 *
	 * @param key    Key to delete in Redis.
	 */
	void del(const std::string& key);

	/**
	 * Perform Redis GET commands in bulk: GET key1; GET key2...
	 *
	 * Pipeget gets multiple keys as a non-atomic operation. More efficient than
	 * getting the keys separately. See:
	 * https://redis.io/topics/mass-insert
	 *
	 * In C++11, this function can be called with brace initialization:
	 * auto values = redis_client.pipeget({"key1", "key2"});
	 * 
	 * @param keys  Vector of keys to get from Redis.
	 * @return      Vector of retrieved values. Optimized with RVO.
	 */
	std::vector<std::string> pipeget(const std::vector<std::string>& keys);

	/**
	 * Perform Redis SET commands in bulk: SET key1 val1; SET key2 val2...
	 *
	 * Pipeset sets multiple keys as a non-atomic operation. More efficient than
	 * setting the keys separately. See:
	 * https://redis.io/topics/mass-insert
	 *
	 * In C++11, this function can be called with brace initialization:
	 * redis_client.pipeset({{"key1", "val1"}, {"key2", "val2"}});
	 * 
	 * @param keyvals  Vector of key-value pairs to set in Redis.
	 */
	void pipeset(const std::vector<std::pair<std::string, std::string>>& keyvals);

	/**
	 * Perform Redis command: MGET key1 key2...
	 *
	 * MGET gets multiple keys as an atomic operation. See:
	 * https://redis.io/commands/mget
	 * 
	 * @param keys  Vector of keys to get from Redis.
	 * @return      Vector of retrieved values. Optimized with RVO.
	 */
	std::vector<std::string> mget(const std::vector<std::string>& keys);

	/**
	 * Perform Redis command: MSET key1 val1 key2 val2...
	 *
	 * MSET sets multiple keys as an atomic operation. See:
	 * https://redis.io/commands/mset
	 *
	 * @param keyvals  Vector of key-value pairs to set in Redis.
	 */
	void mset(const std::vector<std::pair<std::string, std::string>>& keyvals);

	/**
 	 * Encode Eigen::MatrixXd as JSON or space-delimited string.
	 *
	 * encodeEigenMatrixJSON():
	 *   [1,2,3,4]     => "[1,2,3,4]"
	 *   [[1,2],[3,4]] => "[[1,2],[3,4]]"
	 *
	 * encodeEigenMatrixString():
	 *   [1,2,3,4]     => "1 2 3 4"
	 *   [[1,2],[3,4]] => "1 2; 3 4"
	 *
	 * encodeEigenMatrix():
	 *   Encodes JSON or space-delimited string depending on JSON_DEFAULT.
	 *
	 * @param matrix  Eigen::MatrixXd to encode.
	 * @return        Encoded string.
	 */
	template<typename Derived>
	static std::string encodeEigenMatrixJSON(const Eigen::MatrixBase<Derived>& matrix);

	template<typename Derived>
	static std::string encodeEigenMatrixString(const Eigen::MatrixBase<Derived>& matrix);

	template<typename Derived>
	static std::string encodeEigenMatrix(const Eigen::MatrixBase<Derived>& matrix) {
#ifdef JSON_DEFAULT
		return encodeEigenMatrixJSON(matrix);
#else  // JSON_DEFAULT
		return encodeEigenMatrixString(matrix);
#endif  // JSON_DEFAULT
	}

	/**
 	 * Decode Eigen::MatrixXd from JSON or space-delimited string.
	 *
	 * decodeEigenMatrixJSON():
	 *   "[1,2,3,4]"     => [1,2,3,4]
	 *   "[[1,2],[3,4]]" => [[1,2],[3,4]]
	 *
	 * decodeEigenMatrixString():
	 *   "1 2 3 4"  => [1,2,3,4]
	 *   "1 2; 3 4" => [[1,2],[3,4]]
	 *
	 * decodeEigenMatrix():
	 *   Decodes both JSON and space-delimited strings.
	 *
	 * @param str  String to decode.
	 * @return     Decoded Eigen::Matrix. Optimized with RVO.
	 */
	static Eigen::MatrixXd decodeEigenMatrixJSON(const std::string& str);

	static Eigen::MatrixXd decodeEigenMatrixString(const std::string& str);

	static Eigen::MatrixXd decodeEigenMatrix(const std::string& str) {
		return (str[0] == '[') ? decodeEigenMatrixJSON(str) : decodeEigenMatrixString(str);
	}

	/**
	 * Get Eigen::MatrixXd from Redis.
	 *
	 * See decodeEigenMatrix() for description of JSON and space-delimited
	 * string formats.
	 *
	 * @param key  Key to get from Redis.
	 * @return     Value as Eigen::MatrixXd.
	 */
	inline Eigen::MatrixXd getEigenMatrixJSON(const std::string& key) {
		return decodeEigenMatrixJSON(get(key));
	}

	inline Eigen::MatrixXd getEigenMatrixString(const std::string& key) {
		return decodeEigenMatrixString(get(key));
	}

	inline Eigen::MatrixXd getEigenMatrix(const std::string& key) {
		return decodeEigenMatrix(get(key));
	}

	/**
	 * Set Eigen::MatrixXd in Redis.
	 *
	 * See encodeEigenMatrix() for description of JSON and space-delimited
	 * string formats.
	 *
	 * @param key    Key to set in Redis.
	 * @param value  Value for key.
	 */
	template<typename Derived>
	inline void setEigenMatrixJSON(const std::string& key, const Eigen::MatrixBase<Derived>& value) {
		set(key, encodeEigenMatrixJSON(value));
	}

	template<typename Derived>
	inline void setEigenMatrixString(const std::string& key, const Eigen::MatrixBase<Derived>& value) {
		set(key, encodeEigenMatrixString(value));
	}

	template<typename Derived>
	inline void setEigenMatrix(const std::string& key, const Eigen::MatrixBase<Derived>& value) {
		set(key, encodeEigenMatrix(value));
	}

};

//Implementation must be part of header for compile time template specialization
template<typename Derived>
std::string RedisClient::encodeEigenMatrixJSON(const Eigen::MatrixBase<Derived>& matrix) {
	std::string s = "[";
	if (matrix.cols() == 1) { // Column vector
		// [[1],[2],[3],[4]] => "[1,2,3,4]"
		for (int i = 0; i < matrix.rows(); ++i) {
			if (i > 0) s.append(",");
			s.append(std::to_string(matrix(i,0)));
		}
	} else { // Matrix
		// [[1,2,3,4]]   => "[1,2,3,4]"
		// [[1,2],[3,4]] => "[[1,2],[3,4]]"
		for (int i = 0; i < matrix.rows(); ++i) {
			if (i > 0) s.append(",");
			// Nest arrays only if there are multiple rows
			if (matrix.rows() > 1) s.append("[");
			for (int j = 0; j < matrix.cols(); ++j) {
				if (j > 0) s.append(",");
				s.append(std::to_string(matrix(i,j)));
			}
			// Nest arrays only if there are multiple rows
			if (matrix.rows() > 1) s.append("]");
		}
	}
	s.append("]");
	return s;
}

template<typename Derived>
std::string RedisClient::encodeEigenMatrixString(const Eigen::MatrixBase<Derived>& matrix) {
	std::string s;
	if (matrix.cols() == 1) { // Column vector
		// [[1],[2],[3],[4]] => "1 2 3 4"
		for (int i = 0; i < matrix.rows(); ++i) {
			if (i > 0) s += " ";
			s += std::to_string(matrix(i,0));
		}
	} else { // Matrix
		// [1,2,3,4]     => "1 2 3 4"
		// [[1,2],[3,4]] => "1 2; 3 4"
		for (int i = 0; i < matrix.rows(); ++i) {
			if (i > 0) s += "; ";
			for (int j = 0; j < matrix.cols(); ++j) {
				if (j > 0) s += " ";
				s += std::to_string(matrix(i,j));
			}
		}
	}
	return s;
}


#endif //REDIS_CLIENT_H
