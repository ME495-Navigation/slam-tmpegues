#ifndef TURTLELIB_GEOMETRY2D_HPP_INCLUDE_GUARD
#define TURTLELIB_GEOMETRY2D_HPP_INCLUDE_GUARD
/// \file
/// \brief Two-dimensional geometric primitives and other mathematical objects

// NOTE: Put additional include files here

#include <format>

// Note: <iosfwd> contains forward definitions for iostream objects
// allowing implementation of custom iostream operators without
// requiring the inclusion of <iostream>, which is a big header file
#include <iosfwd>
namespace turtlelib
{
/// \brief a 2-Dimensional Point
struct Point2D
{
  /// \brief the x coordinate
  double x = 0.0;

  /// \brief the y coordinate
  double y = 0.0;
};

/// \brief Input a 2 dimensional point
///   You should be able to read vectors entered as follows:
///   "(x, y)" or "x y"  (Not including the "").
/// \param is An istream from which to read
/// \param p [out] The Point2D object that will store the input
/// \returns A reference to is. An error flag is set on the stream if the input cannot be parsed.
///
/// Hint: The following methods may be useful:
/// https://en.cppreference.com/w/cpp/io/basic_istream/peek
///  .peek() looks at the next unprocessed character in the buffer without removing it
/// https://en.cppreference.com/w/cpp/io/basic_istream/get
///  .get() removes the next unprocessed character from the buffer.
///
/// You can also re-use operator>> from other types.
///
/// The way input with istreams works is (more or less):
/// What the user types is stored in a buffer until the user enters a newline (by pressing enter).
/// The iostream methods then process the data in this buffer character-by-character.
/// Typically, each character is examined and then removed from the buffer automatically.
/// If the characters don't match what is expected (e.g., we expected an int but 'q' is encountered)
/// an error flag is set on the stream object (e.g., std::cin).
///
/// If you find yourself writing more than 30 or so lines for this function, you are likely
/// on the wrong track.
std::istream & operator>>(std::istream & is, Point2D & p);

/// \brief A 2-Dimensional Vector
struct Vector2D
{
  /// \brief scale a vector by a scalar
  /// \param rhs The scalar to scale the vector by
  /// \return a reference to the scaled vector
  template<typename T>
  Vector2D & operator*=(const T & rhs)
  {
    x *= rhs;
    y *= rhs;

    return *this;
  }
  /// \brief Add a vector to another
  /// \param rhs The vector to add to the other vector
  /// \return A reference to the summed vector
  Vector2D & operator+=(const Vector2D & rhs);

  /// \brief Subtract a vector from another
  /// \param rhs The vector to subtract from the other vector
  /// \return A reference to the subtracted vector
  Vector2D & operator-=(const Vector2D & rhs);

  /// \brief the x coordinate
  double x = 0.0;

  /// \brief the y coordinate
  double y = 0.0;
};

/// \brief Subtracting one point from another yields a vector
/// \param head point corresponding to the head of the vector
/// \param tail point corresponding to the tail of the vector
/// \return a vector that points from p1 to p2
/// NOTE: this operator is not implemented in terms of -=
/// because subtracting two Point2D yields a Vector2D not a Point2D
Vector2D operator-(const Point2D & head, const Point2D & tail);

/// \brief Adding a vector to a point yields a new point displaced by the vector
/// \param tail The origin of the vector's tail
/// \param disp The displacement vector
/// \return the point reached by displacing by disp from tail
/// NOTE: this is not implemented in terms of += because of the different types
Point2D operator+(const Point2D & tail, const Vector2D & disp);

/// \brief Multiplying a scalar to a vector yields a scaled vector
/// \param scalar value to scale the vector by
/// \param vect the vector to be scaled
/// \return the scaled vector
template<typename T>
Vector2D operator*(const T & scalar, Vector2D vect)
{
  return vect *= scalar;
}

/// \brief Multiplying a scalar to a vector yields a scaled vector
/// \param vect the vector to be scaled
/// \param scalar value to scale the vector by
/// \return the scaled vector
template<typename T>
Vector2D operator*(Vector2D vect, const T & scalar)
{
  return vect *= scalar;
}

/// \brief Add two vectors to yield a single vector
/// \param lhs a vector to be added
/// \param rhs the other vector to be added
/// \return the resultant vector sum
Vector2D operator+(Vector2D lhs, Vector2D & rhs);

/// \brief Subtract two vectors to yield a single vector
/// \param lhs The vector to be subtracted from
/// \param rhs The vector to be subtracted
/// \return the resultant vector difference
Vector2D operator-(Vector2D lhs, Vector2D & rhs);

/// \brief output a 2 dimensional vector as [xcomponent, ycomponent]
/// \param os - stream to output to
/// \param v - the vector to print
/// Note: Here to demonstrate the old method of custom output
/// std::format is very recent so std::ostream is commonly used.
/// DO NOT implement in terms of std::format, this is for you
/// to have exposure to code that does not have std::format available.
std::ostream & operator<<(std::ostream & os, const Vector2D & v);

/// \brief output a 2 dimensional point as (xcomponent, ycomponent)
/// \param os - stream to output to
/// \param v - the point to print
std::ostream & operator<<(std::ostream & os, const Point2D & p);

/// \brief input a 2 dimensional vector
///   You should be able to read vectors entered as follows:
///   "[x, y]" or "x y" (not including the "")
/// \param is An istream from which to read
/// \param v [out] - output vector
/// \returns a reference to the istream, with any error flags set if
/// a parsing error occurs
std::istream & operator>>(std::istream & is, Vector2D & v);

/// \brief Return a unit vector in the direction of v
/// \param in The vector to normalize
/// \return The normalized vector.
/// \throws std::invalid_input if in is the zero vector
Vector2D normalize(Vector2D in);

/// \brief Return the dot product of the two vectors
/// \param vect1 The 1st vector
/// \param vect2 The 2nd vector
/// \return The dot product
double dot(const Vector2D & vect1, const Vector2D & vect2);

/// \brief Calculate the magnitude of the vector
/// \param vect The vector to calculate the magnitude of
/// \return The magnitude of vect
double magnitude(Vector2D & vect);

/// \brief Calculate the shortest angle between two vectors
/// \param vect1 The 1st vector
/// \param vect2 The 2nd vector
/// \return The shortest angle between the two vectors (radians)
double angle(Vector2D & vect1, Vector2D & vect2);
}  // namespace turtlelib

/// \brief A Formatter for 2D points
/// The output is "(x, y)"
/// All floating-point format specifiers are honored and applied to both x and y.
template<class CharT>
class std::formatter<turtlelib::Point2D, CharT>
{
  // static_assert(std::formattable<turtlelib::Point2D>);
  // constexpr auto parse(auto &parse_ctx);

  // auto format(const MyType &t, auto &fmt_ctx) const;

  // std::formatter<T, CharT> T_formatter; // Member variable
  // constexpr auto parse(auto &parse_ctx)
  // {
  //     return T_formatter.parse(parse_ctx);
  // }
};

/// \brief A formatter for Vector2D
/// All double format-spec specifiers apply to each number in the vector
/// The vector is output as [x, y]
template<class CharT>
class std::formatter<turtlelib::Vector2D, CharT>
{
};
#endif
