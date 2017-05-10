#include "RedisClient.h"

std::string RedisClient::get(const std::string& key) {
	// Call GET command
	reply_ = (redisReply *)redisCommand(context_, "GET %s", key.c_str());

	// Check for errors
	if (reply_ == nullptr) {
		throw(std::runtime_error("GET '" + key + "' failed."));
	} else if (reply_->type != REDIS_REPLY_STRING) {
		freeReplyObject(reply_);
		throw(std::runtime_error("GET '" + key + "' returned non-string value."));
	}

	// Return value
	std::string value = reply_->str;
	freeReplyObject(reply_);
	return value;
}

void RedisClient::set(const std::string& key, const std::string& value) {
	// Call SET command
	reply_ = (redisReply *)redisCommand(context_, "SET %s %s", key.c_str(), value.c_str());

	// Check for errors
	if (reply_ == nullptr) {
		throw(std::runtime_error("SET '" + key + "' '" + value + "' failed."));
	} else if (reply_->type == REDIS_REPLY_ERROR) {
		freeReplyObject(reply_);
		throw(std::runtime_error("SET '" + key + "' '" + value + "' failed."));
	}

	freeReplyObject(reply_);
}

std::vector<std::string> RedisClient::mget(const std::vector<std::string>& keys) {
	// Prepare key list
	std::vector<const char *> argv = {"MGET"};
	for (const auto& key : keys) {
		argv.push_back(key.c_str());
	}

	// Call MGET command with variable argument formatting
	reply_ = (redisReply *)redisCommandArgv(context_, argv.size(), &argv[0], nullptr);

	// Check for errors
	if (reply_ == nullptr) {
		throw(std::runtime_error("MGET command failed."));
	} else if (reply_->type != REDIS_REPLY_ARRAY) {
		freeReplyObject(reply_);
		throw(std::runtime_error("MGET command failed."));
	}

	// Collect values
	std::vector<std::string> values;
	for (size_t i = 0; i < reply_->elements; i++) {
		if (reply_->element[i]->type != REDIS_REPLY_STRING) {
			freeReplyObject(reply_);
			throw(std::runtime_error("MGET command returned non-string values."));
		}

		values.push_back(std::string(reply_->element[i]->str));
	}

	// Return values
	freeReplyObject(reply_);
	return values;
}

void RedisClient::mset(const std::vector<std::pair<std::string, std::string>>& keyvals) {
	// Prepare key-value list
	std::vector<const char *> argv = {"MSET"};
	for (const auto& keyval : keyvals) {
		argv.push_back(keyval.first.c_str());
		argv.push_back(keyval.second.c_str());
	}

	// Call MSET command with variable argument formatting
	reply_ = (redisReply *)redisCommandArgv(context_, argv.size(), &argv[0], nullptr);

	// Check for errors
	if (reply_ == nullptr) {
		throw(std::runtime_error("MSET command failed."));
	} else if (reply_->type == REDIS_REPLY_ERROR) {
		freeReplyObject(reply_);
		throw(std::runtime_error("MSET command failed."));
	}

	freeReplyObject(reply_);
}
