
#ifndef SIGFAULTDEBUG_H_
#define SIGFAULTDEBUG_H_

#include "FileSystem.h"
#include <exception>
#include <signal.h>
#include <sstream>
#include <stdexcept>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/syscall.h>
#include <unistd.h>

#include <cxxabi.h>
#include <execinfo.h>

namespace segfaultdebug
{
std::string exec(const char* cmd)
{
    FILE* pipe = popen(cmd, "r");
    if (!pipe)
        return "ERROR";
    char buffer[128];
    std::string result = "";
    while (!feof(pipe)) {
        if (fgets(buffer, 128, pipe) != NULL)
            result += buffer;
    }
    pclose(pipe);
    return result;
}

using std::runtime_error;

class SegFaultException : public runtime_error
{
  public:
    SegFaultException(std::string s)
        : runtime_error(s.c_str())
    {
    }
    /*const char* what() const throw()
    {
      return "";//stacktrace.c_str();
    }
  private:
    std::string stacktrace;*/
};

/** Print a demangled stack backtrace of the caller function to FILE* out. */
static inline std::string get_stacktrace(int startindex = 1)
{
    static int max_frames = 64;
    std::stringstream ss;

    ss << "Stacktrace:\n";

    // storage array for stack trace address data
    void* addrlist[max_frames + 1];

    // retrieve current stack addresses
    int addrlen = backtrace(addrlist, sizeof(addrlist) / sizeof(void*));

    if (addrlen == 0) {
        ss << "  <empty, possibly corrupt>\n";
        return ss.str();
    }

    // resolve addresses into strings containing "filename(function+address)",
    // this array must be free()-ed
    char** symbollist = backtrace_symbols(addrlist, addrlen);

    // allocate string which will be filled with the demangled function name
    size_t funcnamesize = 256;
    char* funcname = (char*)malloc(funcnamesize);

    // iterate over the returned symbol lines. skip the first, it is the
    // address of this function.
    for (int i = startindex; i < addrlen; i++) {
        char *begin_name = 0, *begin_offset = 0, *end_offset = 0;

        // find parentheses and +address offset surrounding the mangled name:
        // ./module(function+0x15c) [0x8048a6d]
        for (char* p = symbollist[i]; *p; ++p) {
            if (*p == '(')
                begin_name = p;
            else if (*p == '+')
                begin_offset = p;
            else if (*p == ')' && begin_offset) {
                end_offset = p;
                break;
            }
        }

        if (begin_name && begin_offset && end_offset && begin_name < begin_offset) {
            *begin_name++ = '\0';
            *begin_offset++ = '\0';
            *end_offset = '\0';

            // mangled name is now in [begin_name, begin_offset) and caller
            // offset in [begin_offset, end_offset). now apply
            // __cxa_demangle():

            int status;
            char* ret = abi::__cxa_demangle(begin_name, funcname, &funcnamesize, &status);
            if (status == 0) {
                funcname = ret; // use possibly realloc()-ed string
                char syscom[256];

                // To work in a library we have to transform the address see here:
                // http://stackoverflow.com/questions/7556045/how-to-map-function-address-to-function-in-so-files
                // major problem: get the starting address of the current lib
                // further we have to format the return string
                // e.g.: This worked addr2line 0x34f3e7 -e /home/endy/cnws/devel/lib/libalica_engine.so
                // Here 0x34f3e7 is computed by: addrlist[i] - libalicaStartAddr (got it from pmap)

                std::string execname(symbollist[i]);
                if (execname.find(".so") != std::string::npos) {
                    std::ifstream ifs("/proc/self/smaps");
                    char blub[1024];
                    std::string startaddr;
                    while (!ifs.eof()) {
                        ifs.getline(blub, 1024);
                        if (strstr(blub, symbollist[i]) != NULL) {
                            if (strstr(blub, "r-xp")) {
                                startaddr = std::string(blub);
                            }
                        }
                    }

                    startaddr = startaddr.substr(0, startaddr.find("-"));
                    void* startadd = nullptr;

                    sscanf(startaddr.c_str(), "%p", (void**)&startadd);
                    sprintf(syscom, "addr2line -e %s %p", symbollist[i], (void*)((char*)addrlist[i] - (char*)startadd));
                    std::string tmp = exec(syscom);
                    ss << "#" << (i - startindex + 1) << " " << addrlist[i] << " " << funcname << " in " << tmp;
                } else {
                    sprintf(syscom, "addr2line -e %s .text %p", symbollist[i], addrlist[i]);
                    std::string tmp = exec(syscom);
                    tmp = tmp.substr(tmp.find("\n") + 1);
                    ss << "#" << (i - startindex + 1) << " " << addrlist[i] << " " << funcname << " in " << tmp;
                }
            } else {
                // demangling failed. Output function name as a C function with
                // no arguments.
                ss << "[" << symbollist[i] << "] " << begin_name << std::endl; //" " << addrlist[i] << std::endl;
            }
        } else {
            // couldn't parse the line? print the whole line.
            ss << symbollist[i] << std::endl;
        }
    }

    free(funcname);
    free(symbollist);
    return ss.str();
}

typedef void (*handler)();

void segfault_sigaction()
{
    segfaultdebug::SegFaultException s(std::string("SegFaultException ") + segfaultdebug::get_stacktrace(4));
    // s.stacktrace = get_stacktrace();
    throw s;
}

handler handler_segv = &segfault_sigaction;
handler handler_fpe = 0;

void handle_segv()
{
    if (segfaultdebug::handler_segv)
        segfaultdebug::handler_segv();
}

void handle_fpe()
{
    if (segfaultdebug::handler_fpe)
        segfaultdebug::handler_fpe();
}
} // namespace segfaultdebug

#ifdef __x86_64__

#ifndef JAVA_SIGNAL_H
#define JAVA_SIGNAL_H 1

#include <signal.h>
#include <sys/syscall.h>

#define HANDLE_SEGV 1
#define HANDLE_FPE 1

#ifdef __GNUC__
#define SUPPRESS_NOT_USED_WARN __attribute__((unused))
#else
#define SUPPRESS_NOT_USED_WARN
#endif

#define SIGNAL_HANDLER(_name) SUPPRESS_NOT_USED_WARN static void _Jv_##_name(int, siginfo_t*, void* _p __attribute__((__unused__)))

#define HANDLE_DIVIDE_OVERFLOW                                                                                                                                 \
    do {                                                                                                                                                       \
        struct ucontext* _uc = (struct ucontext*)_p;                                                                                                           \
        volatile struct sigcontext* _sc = (struct sigcontext*)&_uc->uc_mcontext;                                                                               \
                                                                                                                                                               \
        register unsigned char* _rip = (unsigned char*)_sc->rip;                                                                                               \
                                                                                                                                                               \
        /* According to the JVM spec, "if the dividend is the negative                                                                                         \
         * integer of largest possible magnitude for the type and the                                                                                          \
         * divisor is -1, then overflow occurs and the result is equal to                                                                                      \
         * the dividend.  Despite the overflow, no exception occurs".                                                                                          \
                                          \                                                                                                                    \
         * We handle this by inspecting the instruction which generated the                                                                                    \
         * signal and advancing ip to point to the following instruction.                                                                                      \
         * As the instructions are variable length it is necessary to do a                                                                                     \
         * little calculation to figure out where the following instruction                                                                                    \
         * actually is.                                                                                                                                        \
                                          \                                                                                                                    \
         */                                                                                                                                                    \
                                                                                                                                                               \
        bool _is_64_bit = false;                                                                                                                               \
                                                                                                                                                               \
        if ((_rip[0] & 0xf0) == 0x40) /* REX byte present.  */                                                                                                 \
        {                                                                                                                                                      \
            unsigned char _rex = _rip[0] & 0x0f;                                                                                                               \
            _is_64_bit = (_rex & 0x08) != 0;                                                                                                                   \
            _rip++;                                                                                                                                            \
        }                                                                                                                                                      \
                                                                                                                                                               \
        /* Detect a signed division of Integer.MIN_VALUE or Long.MIN_VALUE.  */                                                                                \
        if (_rip[0] == 0xf7) {                                                                                                                                 \
            bool _min_value_dividend = false;                                                                                                                  \
            unsigned char _modrm = _rip[1];                                                                                                                    \
                                                                                                                                                               \
            if (((_modrm >> 3) & 7) == 7) {                                                                                                                    \
                if (_is_64_bit)                                                                                                                                \
                    _min_value_dividend = (_sc->rax == 0x8000000000000000L);                                                                                   \
                else                                                                                                                                           \
                    _min_value_dividend = ((_sc->rax & 0xffffffff) == 0x80000000);                                                                             \
            }                                                                                                                                                  \
                                                                                                                                                               \
            if (_min_value_dividend) {                                                                                                                         \
                unsigned char _rm = _modrm & 7;                                                                                                                \
                _sc->rdx = 0; /* the remainder is zero */                                                                                                      \
                switch (_modrm >> 6) {                                                                                                                         \
                case 0:           /* register indirect */                                                                                                      \
                    if (_rm == 5) /* 32-bit displacement */                                                                                                    \
                        _rip += 4;                                                                                                                             \
                    if (_rm == 4) /* A SIB byte follows the ModR/M byte */                                                                                     \
                        _rip += 1;                                                                                                                             \
                    break;                                                                                                                                     \
                case 1: /* register indirect + 8-bit displacement */                                                                                           \
                    _rip += 1;                                                                                                                                 \
                    if (_rm == 4) /* A SIB byte follows the ModR/M byte */                                                                                     \
                        _rip += 1;                                                                                                                             \
                    break;                                                                                                                                     \
                case 2: /* register indirect + 32-bit displacement */                                                                                          \
                    _rip += 4;                                                                                                                                 \
                    if (_rm == 4) /* A SIB byte follows the ModR/M byte */                                                                                     \
                        _rip += 1;                                                                                                                             \
                    break;                                                                                                                                     \
                case 3:                                                                                                                                        \
                    break;                                                                                                                                     \
                }                                                                                                                                              \
                _rip += 2;                                                                                                                                     \
                _sc->rip = (unsigned long)_rip;                                                                                                                \
                return;                                                                                                                                        \
            }                                                                                                                                                  \
        }                                                                                                                                                      \
    } while (0)

extern "C" {
struct kernel_sigaction
{
    void (*k_sa_sigaction)(int, siginfo_t*, void*);
    unsigned long k_sa_flags;
    void (*k_sa_restorer)(void);
    sigset_t k_sa_mask;
};
}

#define MAKE_THROW_FRAME(_exception)

#define RESTORE(name, syscall) RESTORE2(name, syscall)
#define RESTORE2(name, syscall)                                                                                                                                \
    asm(".text\n"                                                                                                                                              \
        ".byte 0  # Yes, this really is necessary\n"                                                                                                           \
        ".align 16\n"                                                                                                                                          \
        "__" #name ":\n"                                                                                                                                       \
        "	movq $" #syscall ", %rax\n"                                                                                                                    \
        "	syscall\n");

/* The return code for realtime-signals.  */
RESTORE(restore_rt, __NR_rt_sigreturn)
void restore_rt(void) asm("__restore_rt") __attribute__((visibility("hidden")));

#define INIT_SEGV                                                                                                                                              \
    do {                                                                                                                                                       \
        struct kernel_sigaction act;                                                                                                                           \
        act.k_sa_sigaction = _Jv_catch_segv;                                                                                                                   \
        sigemptyset(&act.k_sa_mask);                                                                                                                           \
        act.k_sa_flags = SA_SIGINFO | 0x4000000;                                                                                                               \
        act.k_sa_restorer = restore_rt;                                                                                                                        \
        syscall(SYS_rt_sigaction, SIGSEGV, &act, NULL, _NSIG / 8);                                                                                             \
    } while (0)

#define INIT_FPE                                                                                                                                               \
    do {                                                                                                                                                       \
        struct kernel_sigaction act;                                                                                                                           \
        act.k_sa_sigaction = _Jv_catch_fpe;                                                                                                                    \
        sigemptyset(&act.k_sa_mask);                                                                                                                           \
        act.k_sa_flags = SA_SIGINFO | 0x4000000;                                                                                                               \
        act.k_sa_restorer = restore_rt;                                                                                                                        \
        syscall(SYS_rt_sigaction, SIGFPE, &act, NULL, _NSIG / 8);                                                                                              \
    } while (0)

/* You might wonder why we use syscall(SYS_sigaction) in INIT_FPE
 * instead of the standard sigaction().  This is necessary because of
 * the shenanigans above where we increment the PC saved in the
 * context and then return.  This trick will only work when we are
 * called _directly_ by the kernel, because linuxthreads wraps signal
 * handlers and its wrappers do not copy the sigcontext struct back
 * when returning from a signal handler.  If we return from our divide
 * handler to a linuxthreads wrapper, we will lose the PC adjustment
 * we made and return to the faulting instruction again.  Using
 * syscall(SYS_sigaction) causes our handler to be called directly
 * by the kernel, bypassing any wrappers.  */

#endif /* JAVA_SIGNAL_H */

#else /* __x86_64__ */

/* This is for the 32-bit subsystem on x86-64.  */

#define sigcontext_struct sigcontext
// may be a 32 bit solution with libjava:
//#include <java-signal-aux.h>

#endif /* __x86_64__ */

#if defined(HANDLE_SEGV) || defined(HANDLE_FPE)

#include <execinfo.h>

/* Unblock a signal.  Unless we do this, the signal may only be sent
   once.  */
static void unblock_signal(int signum __attribute__((__unused__)))
{
#ifdef _POSIX_VERSION
    sigset_t sigs;
    sigemptyset(&sigs);
    sigaddset(&sigs, signum);
    sigprocmask(SIG_UNBLOCK, &sigs, NULL);
#endif
}
#endif
#ifdef __x86_64__

SIGNAL_HANDLER(catch_segv)
{
    unblock_signal(SIGSEGV);
    MAKE_THROW_FRAME(nullp);
    segfaultdebug::handle_segv();
}

SIGNAL_HANDLER(catch_fpe)
{
    unblock_signal(SIGFPE);
#ifdef HANDLE_DIVIDE_OVERFLOW
    HANDLE_DIVIDE_OVERFLOW;
#else
    MAKE_THROW_FRAME(arithexception);
#endif
    segfaultdebug::handle_fpe();
}
#endif // __x86_64__

namespace segfaultdebug
{
void init_segfault_exceptions()
{
#ifdef __x86_64__
    INIT_SEGV;
#endif // __x86_64__
}
} // namespace segfaultdebug

#endif
