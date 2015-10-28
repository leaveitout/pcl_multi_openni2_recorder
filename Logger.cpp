#include "Logger.h"

namespace Logger {

    namespace {
        boost::shared_ptr<boost::mutex> io_mutex_;

        void initialize() {
            io_mutex_.reset(new boost::mutex());
        }
    }

    boost::once_flag once;

    void log(Logger::Level level, const char *format, ...) {
        boost::call_once(&initialize, once);

        if (!pcl::console::isVerbosityLevelEnabled((pcl::console::VERBOSITY_LEVEL) level)) return;
        FILE *stream = (level == WARN || level == ERROR) ? stderr : stdout;
        switch (level) {
            case DEBUG:
                change_text_color(stream, TT_RESET, TT_GREEN);
                break;
            case WARN:
                change_text_color(stream, TT_BRIGHT, TT_YELLOW);
                break;
            case ERROR:
                change_text_color(stream, TT_BRIGHT, TT_RED);
                break;
            case ALWAYS:
            case INFO:
            case VERBOSE:
            default:
                break;
        }

        va_list ap;
        va_start (ap, format);
        {
            boost::mutex::scoped_lock lock(*io_mutex_);
            vfprintf(stream, format, ap);
        }
        va_end (ap);

        reset_text_color(stream);
    }
}
