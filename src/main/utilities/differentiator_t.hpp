//
// Created by ydrml on 2019/8/3.
//

#ifndef PM1_SDK_DIFFERENTIATOR_T_HPP
#define PM1_SDK_DIFFERENTIATOR_T_HPP


#include <functional>

/**
 * 条件更新器
 * @tparam t 存储类型
 */
template<class t>
struct differentiator_t {
    /**
     * 更新谓词
     */
    using update_predicate_t = std::function<bool(const t &, const t &)>;
    
    t                  memory;
    update_predicate_t default_predicate;
    
    /**
     * 用默认谓词更新
     * @param new_value [in]  新值
     * @param old_value [out] 旧值
     * @return 是否发生更新
     */
    bool update(const t &new_value, t &old_value) {
        if (!default_predicate(memory, new_value))
            return false;
        
        old_value = memory;
        memory    = new_value;
        return true;
    }
    
    /**
     * 用自定义谓词更新
     * @param new_value [in]  新值
     * @param old_value [out] 旧值
     * @param predicate [in]  自定义更新谓词
     * @return 是否发生更新
     */
    bool update(const t &new_value, t &old_value, const update_predicate_t &predicate) {
        if (!predicate(memory, new_value))
            return false;
        
        old_value = memory;
        memory    = new_value;
        return true;
    }
};

/**
 * 生成无默认谓词的条件更新器
 */
template<class t>
constexpr differentiator_t<t> make_blank_differentiator(const t &init) {
    return {init, [](const t &, const t &) { return true; }};
}


#endif //PM1_SDK_DIFFERENTIATOR_T_HPP
