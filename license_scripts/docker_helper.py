from hostid import hostid_to_ip
from read_lic import read_lic
from pathlib import Path


def main():
    script_dir = Path(__file__).parent.absolute()
    lic_dir = script_dir.parent / "licenses"

    # check if lic_dir exists
    if not lic_dir.exists():
        raise ValueError(f'"{lic_dir}" does not exist')

    # get first .lic file in lic_dir
    lic_files = [f for f in lic_dir.iterdir() if f.suffix == ".lic"]
    if len(lic_files) < 1:
        raise ValueError(f'No .lic files found in "{lic_dir}"')
    
    lic_file = lic_files[0]
    print(f'Opening "{lic_file}"')
    hostname, hostid = read_lic(lic_file)

    print("Add the following parameters to your docker run command:")
    print(f"--hostname {hostname}")
    print(f"--ip {hostid_to_ip(hostid)}")


if __name__ == "__main__":
    main()