fn main() {
    println!("cargo:rustc-link-search=native=/home/ubuntu2204/dora-main/target/release");
    println!("cargo:rustc-link-lib=static=dora_node_api_c");
    println!("cargo:rustc-link-lib=m");
    println!("cargo:rustc-link-lib=rt");
    println!("cargo:rustc-link-lib=dl");
    println!("cargo:rustc-link-lib=pthread");
}
