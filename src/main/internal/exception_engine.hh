//
// Created by User on 2019/4/9.
//

#ifndef PM1_SDK_EXCEPTION_ENGINE_HH
#define PM1_SDK_EXCEPTION_ENGINE_HH


#include <shared_mutex>
#include <unordered_map>

namespace autolabor {
	class exception_engine {
		mutable std::shared_mutex               mutex;
		std::unordered_map<size_t, std::string> map;
		
	public:
		void set(size_t id, const std::string &text);
		
		void remove(size_t id);
		
		void clear();
		
		const char *operator[](size_t id) const;
	};
}


#endif //PM1_SDK_EXCEPTION_ENGINE_HH
