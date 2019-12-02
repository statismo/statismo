// Copyright 2019 Ken Avolic <kenavolic@none.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <string>
#include <variant>
#include <vector>

///
/// Light program options (lpo) aims to provide
/// simple arg parsing without the need to include
/// all boost library
///

namespace lpo
{

namespace details
{
template <class T>
struct always_false : std::false_type
{};

template <char>
struct word_delimiter : public std::string
{};

template <char D>
std::istream &
operator>>(std::istream & is, word_delimiter<D> & output)
{
  std::getline(is, output, D);
  return is;
}

template <char D>
inline auto
split(const std::string & in)
{
  std::istringstream       iss(in);
  std::vector<std::string> vec(std::istream_iterator<word_delimiter<D>>{ iss },
                               std::istream_iterator<word_delimiter<D>>{});
  return vec;
}

template <typename T>
inline T
lexical_cast(const std::string &)
{
  throw std::bad_cast();
}

template <>
inline std::string
lexical_cast<std::string>(const std::string & str)
{
  return str;
}

template <>
inline bool
lexical_cast<bool>(const std::string & str)
{
  if (str == "True" || str == "true" || str == "1")
  {
    return true;
  }
  else if (str == "False" || str == "false" || str == "0")
  {
    return false;
  }

  throw std::bad_cast();
}

template <>
inline std::vector<std::string>
lexical_cast<std::vector<std::string>>(const std::string & str)
{
  return split<','>(str);
}

template <>
inline double
lexical_cast<double>(const std::string & str)
{
  return std::stod(str);
}

template <>
inline float
lexical_cast<float>(const std::string & str)
{
  return std::stof(str);
}

template <>
inline int
lexical_cast<int>(const std::string & str)
{
  return std::stoi(str);
}

template <>
inline long
lexical_cast<long>(const std::string & str)
{
  return std::stol(str);
}

template <>
inline long long
lexical_cast<long long>(const std::string & str)
{
  return std::stoll(str);
}

template <>
inline unsigned int
lexical_cast<unsigned int>(const std::string & str)
{
  return std::stoul(str);
}

template <>
inline unsigned long
lexical_cast<unsigned long>(const std::string & str)
{
  return std::stoul(str);
}

template <>
inline unsigned long long
lexical_cast<unsigned long long>(const std::string & str)
{
  return std::stoull(str);
}

inline bool
ensure_alphanum_ext(const std::string & str)
{
  return std::all_of(
    std::cbegin(str), std::cend(str), [](char c) { return std::isalnum(c) || (c == '_') || (c == '-'); });
}

std::string
extract_filename(const std::string & s)
{
  char sep = '/';
#ifdef _WIN32
  sep = '\\';
#endif
  std::size_t i = s.rfind(sep, s.length());
  if (i != std::string::npos)
  {
    return (s.substr(i + 1, s.length() - i));
  }
  return s;
}
} // namespace details

template <typename... Ts>
class program_options
{
private:
  template <typename T, typename = void>
  struct opt
  {
    using value_type = std::decay_t<T>;

    std::string name;
    std::string short_name;
    std::string desc;
    T *         val;
    T           default_val;
  };

  template <typename T>
  struct opt<T, std::enable_if_t<std::is_arithmetic_v<T>>>
  {
    using value_type = std::decay_t<T>;

    std::string name;
    std::string short_name;
    std::string desc;
    T *         val;
    T           default_val;
    T           min{ std::numeric_limits<T>::min() };
    T           max{ std::numeric_limits<T>::max() };
  };

  template <typename T, typename = void>
  struct pos_opt
  {
    using value_type = std::decay_t<T>;
    std::string desc;
    T *         val;
  };

  template <typename T>
  struct pos_opt<T, std::enable_if_t<std::is_arithmetic_v<T>>>
  {
    using value_type = std::decay_t<T>;
    std::string desc;
    T *         val;
    T           min{ std::numeric_limits<T>::min() };
    T           max{ std::numeric_limits<T>::max() };
  };

  using flag_opt_t = opt<bool>;
  using opt_t = std::variant<opt<Ts>...>;
  using pos_opt_t = std::variant<pos_opt<Ts>...>;

public:
  ///
  /// @brief Set global desc printed with help
  ///
  explicit program_options(const std::string & prog, const std::string & desc)
    : m_prog{ details::extract_filename(prog) }
    , m_global_desc{ desc }
  {}

  ///
  /// @brief Register a new flag (i.e. options set or not without value like
  /// --version -v)
  ///
  program_options &
  add_flag(flag_opt_t && flag);

  ///
  /// @brief Register a new value option
  ///
  template <typename T>
  program_options &
  add_opt(opt<T> && value_opt, bool mandatory = false);

  ///
  /// @brief Register a new positional arg
  ///
  template <typename T>
  program_options &
  add_pos_opt(pos_opt<T> && pos_opt);

  ///
  /// @brief Check if a given option was parsed (flag or value option)
  ///
  bool
  has(const std::string & opt_name) const;

  ///
  /// @brief parse program args
  ///
  bool
  parse(int argc, char ** argv);

  void
  print(std::ostream & os) const;

private:
  template <typename T>
  bool
  has_arg_match(const T & opt, const std::string & arg) const
  {
    return (("--" + opt.name) == arg) || (!opt.short_name.empty() && (("-" + opt.short_name) == arg));
  }

  template <typename Opt>
  bool
  in_range(const Opt & o) const
  {
    return (*(o.val) >= o.min) && (*(o.val) <= o.max);
  }

  bool
  has_opt(const std::string & name, const std::string & short_name) const
  {
    return std::any_of(std::cbegin(m_check_list), std::cend(m_check_list), [&](const auto & n) {
      return (name == n.first || (!short_name.empty() && short_name == n.second));
    });
  }

  void
  add_opt(const std::string & name, const std::string & short_name)
  {
    if (has_opt(name, short_name))
    {
      throw std::runtime_error("duplicate option name or short name for option " + name);
    }

    if (name.empty())
    {
      throw std::runtime_error("invalid empty option name");
    }

    if (!details::ensure_alphanum_ext(name))
    {
      throw std::runtime_error("invalid non alphanum option name " + name);
    }

    if (!short_name.empty() && !details::ensure_alphanum_ext(short_name))
    {
      throw std::runtime_error("invalid non alphanum option short name " + short_name);
    }

    m_check_list[name] = short_name;
  }

  std::string                        m_prog;
  std::string                        m_global_desc;
  std::map<std::string, std::string> m_check_list;
  std::map<std::string, bool>        m_mandatory_map;
  std::vector<flag_opt_t>            m_flag_list;
  std::vector<opt_t>                 m_opt_list;
  std::vector<pos_opt_t>             m_pos_list;
};

// Print program options
template <typename T>
inline auto
operator<<(std::ostream & os, const T & obj) -> decltype(std::declval<T>().print(os), os)
{
  obj.print(os);
  return os;
}

// Implementation

template <typename... Ts>
program_options<Ts...> &
program_options<Ts...>::add_flag(flag_opt_t && flag)
{
  add_opt(flag.name, flag.short_name);

  *(flag.val) = false;
  m_flag_list.push_back(std::move(flag));
  return *this;
}

template <typename... Ts>
template <typename T>
program_options<Ts...> &
program_options<Ts...>::add_opt(opt<T> && value_opt, bool mandatory)
{
  add_opt(value_opt.name, value_opt.short_name);

  if (mandatory)
  {
    m_mandatory_map[value_opt.name] = false;
  }

  auto tmp_opt = std::move(value_opt);
  *(tmp_opt.val) = tmp_opt.default_val;
  m_opt_list.push_back(std::move(tmp_opt));

  return *this;
}

template <typename... Ts>
template <typename T>
program_options<Ts...> &
program_options<Ts...>::add_pos_opt(pos_opt<T> && pos_opt)
{
  m_pos_list.push_back(std::move(pos_opt));
  return *this;
}

template <typename... Ts>
bool
program_options<Ts...>::parse(int argc, char ** argv)
{
  std::vector<std::string> args;
#if defined _WIN32
  int                                              argCount;
  LPWSTR *                                         argListUtf16 = CommandLineToArgvW(GetCommandLineW(), &argCount);
  std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
  for (int i = 0; i < argCount; i++)
  {
    args.push_back(converter.to_bytes(argListUtf16[i]));
  }
#else
  args = std::vector<std::string>(argv, argv + argc);
#endif

  // start at 1, skip program name
  auto args_count{ 1 };

  // This loop parses flags and value options
  for (args_count = 1; args_count < argc; ++args_count)
  {
    // retrieve current arg
    auto arg = args[args_count];

    // handle help args automatically
    if (arg == "-h" || arg == "--help")
    {
      print(std::cout);
      return false;
    }

    bool match{ false };

    // check if it is a flag
    match = std::any_of(std::cbegin(m_flag_list), std::cend(m_flag_list), [&arg, this](auto & p) {
      bool match = has_arg_match(p, arg);
      if (match)
      {
        *(p.val) = true;
      }
      return match;
    });

    if (match)
    {
      continue;
    }

    // no match check for a value match
    for (auto & v : m_opt_list)
    {
      try
      {
        std::visit(
          [&arg, &match, &args_count, &args, &argc, this](auto && opt) {
            using T = typename std::decay_t<decltype(opt)>::value_type;
            if (has_arg_match(opt, arg))
            {
              if (args_count == (argc - 1))
              {
                throw std::runtime_error("missing value for option <" + opt.name + ">");
              }
              auto arg = args[++args_count];
              try
              {
                *(opt.val) = details::lexical_cast<T>(arg);
              }
              catch (...)
              {
                throw std::runtime_error("invalid value for option <" + opt.name + ">");
              }

              if constexpr (std::is_arithmetic_v<T>)
              {
                if (!in_range(opt))
                {
                  throw std::runtime_error("out of range value for option <" + opt.name + ">");
                }
              }

              if (m_mandatory_map.find(opt.name) != std::cend(m_mandatory_map))
              {
                m_mandatory_map[opt.name] = true;
              }

              match = true;
            }
          },
          v);
      }
      catch (const std::runtime_error & ex)
      {
        std::cerr << "[-] value option parsing failure: " << ex.what() << std::endl;
        return false;
      }
      catch (...)
      {
        std::cerr << "[-] value option unknown parsing failure" << std::endl;
        return false;
      }

      if (match)
      {
        break;
      }
    }

    if (!match)
    {
      if (arg[0] == '-')
      {
        std::cerr << "[-] unknown value option " << arg << std::endl;
        return false;
      }

      break;
    }
  }

  // check all mandtory options are set
  // Note: Disabled as it is not compatible with boost option behavior
  // std::string tmp_opt;
  // if (!std::all_of(std::cbegin(m_mandatory_map), std::cend(m_mandatory_map),
  //                  [&tmp_opt](const auto &p) {
  //                    tmp_opt = p.first;
  //                    return p.second;
  //                  })) {
  //   std::cerr << "[-] missing mandatory option " << tmp_opt << std::endl;
  //   return false;
  // }

  if (static_cast<unsigned>(argc - args_count) != m_pos_list.size())
  {
    std::cerr << "[-] missing positionnal args" << std::endl;
    return false;
  }

  for (unsigned i = 0; i < m_pos_list.size(); ++args_count, ++i)
  {
    auto arg = args[args_count];

    try
    {
      std::visit(
        [&arg, &i, this](auto && opt) {
          using T = typename std::decay_t<decltype(opt)>::value_type;
          try
          {
            *(opt.val) = details::lexical_cast<T>(arg);
          }
          catch (...)
          {
            throw std::runtime_error("invalid value for pos argument number " + std::to_string(i + 1));
          }

          if constexpr (std::is_arithmetic_v<T>)
          {
            if (!in_range(opt))
            {
              throw std::runtime_error("out of range value for pos argument number " + std::to_string(i + 1));
            }
          }
        },
        m_pos_list[i]);
    }
    catch (const std::runtime_error & ex)
    {
      std::cerr << "[-] positionnal arg parsing failure: " << ex.what() << std::endl;
      return false;
    }
    catch (...)
    {
      std::cerr << "[-] positionnal arg unknown parsing failure" << std::endl;
      return false;
    }
  }

  return true;
}
template <typename... Ts>
void
program_options<Ts...>::print(std::ostream & os) const
{
  os << (m_global_desc.empty() ? "Program help:" : m_global_desc) << "\n=============\n\n";
  os << "Usage: " << m_prog << " <options> ";

  for (unsigned i = 0; i < m_pos_list.size(); ++i)
  {
    std::visit([&os, &i](auto &&) { os << "[ARG" << std::to_string(i + 1) << "] "; }, m_pos_list[i]);
  }

  os << "\n\n";
  for (unsigned i = 0; i < m_pos_list.size(); ++i)
  {
    std::visit([&os, &i](auto && opt) { os << "ARG" << std::to_string(i + 1) << ":\n\n"
                                           << opt.desc << std::endl; },
               m_pos_list[i]);
  }

  os << "\n\n";
  os << "Allowed options:\n----------------\n" << std::endl;
  os << "--help/-h:\n\n"
     << "Display help\n\n"
     << std::endl;

  for (auto & f : m_flag_list)
  {
    os << "--" << f.name;

    if (!f.short_name.empty())
    {
      os << "/-" << f.short_name;
    }

    os << ":\n\n" << f.desc << "\n\n" << std::endl;
  }

  for (auto & o : m_opt_list)
  {
    std::visit(
      [&os, this](auto && opt) {
        os << "--" << opt.name;

        if (!opt.short_name.empty())
        {
          os << "/-" << opt.short_name;
        }

        os << " {val}:";

        if (m_mandatory_map.count(opt.name))
        {
          os << " [mandatory]";
        }

        os << "\n\n" << opt.desc << "\n\n" << std::endl;
      },
      o);
  }
}
} // namespace lpo