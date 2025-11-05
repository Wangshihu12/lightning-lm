if (MSVC)
add_library(miao_utils STATIC
        sampler.cc
)
else ()
add_library(miao_utils SHARED
        sampler.cc
)
endif ()
