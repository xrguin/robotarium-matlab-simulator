#!/usr/bin/env python3
"""
Python Package Availability Check for Robotarium Platform
This script checks what Python packages and versions are available
for potential neural network prediction implementation.
"""

import sys
import platform
import subprocess
import json
from datetime import datetime

def check_package(package_name, import_name=None):
    """Check if a package is installed and get its version."""
    if import_name is None:
        import_name = package_name
    
    try:
        module = __import__(import_name)
        version = getattr(module, '__version__', 'Unknown version')
        return {'installed': True, 'version': version}
    except ImportError:
        return {'installed': False, 'version': None}

def run_command(command):
    """Run a shell command and return the output."""
    try:
        result = subprocess.run(command, shell=True, capture_output=True, text=True)
        return result.stdout.strip()
    except Exception as e:
        return f"Error: {str(e)}"

def main():
    print("=" * 60)
    print("PYTHON ENVIRONMENT CHECK FOR ROBOTARIUM PLATFORM")
    print("=" * 60)
    
    # System information
    system_info = {
        'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
        'python_version': sys.version,
        'platform': platform.platform(),
        'architecture': platform.architecture(),
        'machine': platform.machine(),
        'processor': platform.processor(),
        'python_executable': sys.executable
    }
    
    print(f"Timestamp: {system_info['timestamp']}")
    print(f"Python Version: {system_info['python_version']}")
    print(f"Platform: {system_info['platform']}")
    print(f"Architecture: {system_info['architecture']}")
    print(f"Machine: {system_info['machine']}")
    print(f"Python Executable: {system_info['python_executable']}")
    print()
    
    # Essential packages for neural networks
    packages_to_check = [
        ('numpy', 'numpy'),
        ('scipy', 'scipy'),
        ('matplotlib', 'matplotlib'),
        ('scikit-learn', 'sklearn'),
        ('pandas', 'pandas'),
        ('tensorflow', 'tensorflow'),
        ('torch', 'torch'),
        ('keras', 'keras'),
        ('pytorch', 'torch'),  # Alternative check
        ('h5py', 'h5py'),
        ('pickle', 'pickle'),
        ('joblib', 'joblib'),
        ('onnx', 'onnx'),
        ('onnxruntime', 'onnxruntime'),
    ]
    
    print("CHECKING NEURAL NETWORK PACKAGES:")
    print("-" * 40)
    
    package_results = {}
    
    for package_name, import_name in packages_to_check:
        result = check_package(package_name, import_name)
        package_results[package_name] = result
        
        status = "✓ INSTALLED" if result['installed'] else "✗ NOT FOUND"
        version_info = f" (v{result['version']})" if result['installed'] and result['version'] != 'Unknown version' else ""
        
        print(f"{package_name:15} : {status}{version_info}")
    
    print()
    
    # Check pip packages
    print("CHECKING PIP PACKAGES:")
    print("-" * 40)
    pip_list = run_command("pip list")
    print(pip_list[:1000] + "..." if len(pip_list) > 1000 else pip_list)
    print()
    
    # Check conda packages (if available)
    print("CHECKING CONDA PACKAGES:")
    print("-" * 40)
    conda_list = run_command("conda list")
    if "Error:" not in conda_list:
        print(conda_list[:1000] + "..." if len(conda_list) > 1000 else conda_list)
    else:
        print("Conda not available")
    print()
    
    # Test basic neural network functionality
    print("TESTING BASIC FUNCTIONALITY:")
    print("-" * 40)
    
    functionality_tests = {}
    
    # Test NumPy
    try:
        import numpy as np
        test_array = np.array([1, 2, 3, 4, 5])
        test_result = np.mean(test_array)
        functionality_tests['numpy_basic'] = {'success': True, 'result': str(test_result)}
        print("NumPy basic operations: ✓ WORKING")
    except Exception as e:
        functionality_tests['numpy_basic'] = {'success': False, 'error': str(e)}
        print(f"NumPy basic operations: ✗ FAILED - {e}")
    
    # Test SciPy
    try:
        import scipy
        from scipy import optimize
        functionality_tests['scipy_basic'] = {'success': True, 'version': scipy.__version__}
        print("SciPy import: ✓ WORKING")
    except Exception as e:
        functionality_tests['scipy_basic'] = {'success': False, 'error': str(e)}
        print(f"SciPy import: ✗ FAILED - {e}")
    
    # Test TensorFlow
    try:
        import tensorflow as tf
        print(f"TensorFlow version: {tf.__version__}")
        # Try to create a simple model
        model = tf.keras.Sequential([tf.keras.layers.Dense(1, input_shape=[1])])
        functionality_tests['tensorflow_model'] = {'success': True, 'version': tf.__version__}
        print("TensorFlow model creation: ✓ WORKING")
    except Exception as e:
        functionality_tests['tensorflow_model'] = {'success': False, 'error': str(e)}
        print(f"TensorFlow: ✗ FAILED - {e}")
    
    # Test PyTorch
    try:
        import torch
        print(f"PyTorch version: {torch.__version__}")
        # Try to create a simple tensor
        x = torch.randn(5, 3)
        functionality_tests['pytorch_tensor'] = {'success': True, 'version': torch.__version__}
        print("PyTorch tensor creation: ✓ WORKING")
    except Exception as e:
        functionality_tests['pytorch_tensor'] = {'success': False, 'error': str(e)}
        print(f"PyTorch: ✗ FAILED - {e}")
    
    # Test scikit-learn
    try:
        from sklearn.neural_network import MLPRegressor
        from sklearn.model_selection import train_test_split
        import numpy as np
        
        # Create dummy data
        X = np.random.random((100, 10))
        y = np.random.random(100)
        
        # Try to create and fit a simple neural network
        mlp = MLPRegressor(hidden_layer_sizes=(50,), max_iter=100, random_state=42)
        X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)
        mlp.fit(X_train, y_train)
        
        functionality_tests['sklearn_mlp'] = {'success': True, 'score': mlp.score(X_test, y_test)}
        print("Scikit-learn MLP: ✓ WORKING")
    except Exception as e:
        functionality_tests['sklearn_mlp'] = {'success': False, 'error': str(e)}
        print(f"Scikit-learn MLP: ✗ FAILED - {e}")
    
    print()
    
    # Compile results
    results = {
        'system_info': system_info,
        'package_results': package_results,
        'functionality_tests': functionality_tests,
        'pip_packages': pip_list,
        'conda_packages': conda_list if "Error:" not in conda_list else None
    }
    
    # Save results to JSON file
    try:
        with open('python_environment_check.json', 'w') as f:
            json.dump(results, f, indent=2)
        print("Results saved to: python_environment_check.json")
    except Exception as e:
        print(f"Failed to save results: {e}")
    
    # Print summary
    print()
    print("SUMMARY:")
    print("-" * 40)
    installed_packages = sum(1 for pkg in package_results.values() if pkg['installed'])
    total_packages = len(package_results)
    
    print(f"Packages found: {installed_packages}/{total_packages}")
    
    # Recommendations
    print()
    print("RECOMMENDATIONS FOR NEURAL NETWORK IMPLEMENTATION:")
    print("-" * 50)
    
    if package_results.get('tensorflow', {}).get('installed'):
        print("✓ TensorFlow available - Can implement TF-based prediction")
    elif package_results.get('torch', {}).get('installed'):
        print("✓ PyTorch available - Can implement PyTorch-based prediction")
    elif package_results.get('sklearn', {}).get('installed'):
        print("✓ Scikit-learn available - Can use MLPRegressor for prediction")
    else:
        print("✗ No major ML frameworks found")
        if package_results.get('numpy', {}).get('installed'):
            print("  → Consider implementing simple neural network from scratch with NumPy")
        else:
            print("  → No suitable packages for neural network implementation")
    
    print()
    print("=" * 60)
    print("CHECK COMPLETE")
    print("=" * 60)

if __name__ == "__main__":
    main()