# Contributing to FRC MATLAB Control Library

Thank you for your interest in contributing! This document provides guidelines for contributing to the project.

## Code of Conduct

- Be respectful and inclusive
- Welcome newcomers and help them learn
- Focus on constructive feedback
- Follow FRC values of Gracious Professionalism

## How to Contribute

### Reporting Bugs

1. Check if the bug has already been reported in [Issues](https://github.com/yourusername/frc-matlab-control-library/issues)
2. If not, create a new issue with:
   - Clear, descriptive title
   - Steps to reproduce
   - Expected vs actual behavior
   - MATLAB version and operating system
   - Minimal code example demonstrating the issue

### Suggesting Features

1. Check [Discussions](https://github.com/yourusername/frc-matlab-control-library/discussions) to see if it's been proposed
2. Create a new discussion describing:
   - The problem your feature solves
   - How it would work
   - Example use cases
   - Any implementation ideas

### Pull Requests

1. Fork the repository
2. Create a new branch: `git checkout -b feature/your-feature-name`
3. Make your changes following our coding standards
4. Test your changes thoroughly
5. Commit with clear messages: `git commit -m "Add feature: description"`
6. Push to your fork: `git push origin feature/your-feature-name`
7. Open a Pull Request with:
   - Description of changes
   - Related issue numbers
   - Test results

## Development Guidelines

### Code Style

- Use clear, descriptive variable names
- Add comments explaining complex logic
- Follow MATLAB naming conventions:
  - Functions: `camelCase`
  - Classes: `PascalCase`
  - Constants: `UPPER_CASE`
- Keep functions focused (single responsibility)
- Maximum line length: 100 characters

### Documentation

- Add function headers with:
  ```matlab
  % FUNCTIONNAME - Brief description
  %
  % Syntax:
  %   output = functionName(input1, input2)
  %
  % Inputs:
  %   input1 - Description
  %   input2 - Description
  %
  % Outputs:
  %   output - Description
  %
  % Example:
  %   result = functionName(10, 'test');
  %
  % See also: RELATEDFUNCTION1, RELATEDFUNCTION2
  ```
- Update relevant documentation files
- Add examples for new features

### Testing

- Test with multiple MATLAB versions when possible
- Verify backward compatibility with R2019a
- Include example usage in comments
- Test edge cases and error conditions

### File Organization

- Place files in appropriate directories:
  - Motor models → `src/motors/`
  - Mechanisms → `src/templates/`
  - Controllers → `src/control/`
  - Simulation → `src/simulation/`
  - System ID → `src/system_id/`
  - Plugins → `src/plugins/`
  - Examples → `examples/`
  - Documentation → `docs/`

## Adding New Features

### New Motor Models

1. Create file in `src/motors/`
2. Follow existing motor model structure
3. Include manufacturer specs in comments
4. Add to motor library documentation

### New Mechanism Templates

1. Create file in `src/templates/`
2. Inherit from appropriate base class if applicable
3. Include working example in file comments
4. Add visualization support
5. Update documentation

### New Plugins

1. Register with `VisualizationSystem`
2. Follow plugin interface conventions
3. Include help text
4. Add example usage
5. Update plugin documentation

## Project Structure

```
src/
├── motors/         - Motor models (KrakenX40, KrakenX60, etc.)
├── templates/      - Mechanism templates
├── control/        - Controller synthesis tools
├── simulation/     - Physics simulation
├── system_id/      - System identification
└── plugins/        - Visualization and validation plugins

examples/           - Working example scripts
docs/              - Documentation files
```

## Commit Message Guidelines

Format:
```
<type>: <subject>

<body>

<footer>
```

Types:
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation changes
- `style`: Code formatting (no functional changes)
- `refactor`: Code restructuring
- `test`: Adding/updating tests
- `chore`: Maintenance tasks

Example:
```
feat: Add NEO motor model

- Implement NEO motor specifications
- Add stall torque and free speed data
- Include efficiency curves
- Add usage example in comments

Closes #42
```

## Questions?

- Open a [Discussion](https://github.com/yourusername/frc-matlab-control-library/discussions) for general questions
- Create an [Issue](https://github.com/yourusername/frc-matlab-control-library/issues) for bugs or feature requests
- Check existing documentation in the `docs/` folder

## Recognition

Contributors will be acknowledged in:
- README.md contributors section
- Release notes
- Project documentation

Thank you for helping make this library better for FRC teams worldwide! 🚀
